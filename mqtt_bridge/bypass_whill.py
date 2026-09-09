#!/usr/bin/env python3
"""bypass_whill.py

WHILL Autonomous Mobility API (Traffic Control) と通信するためのHTTP通信ノード。

役割としては「HTTP通信ノード」だが、実際には以下も担う：
- /vehicle_current(waypointId)を監視し、設定ファイル(whill.yaml)で定義された Conflict Zone の
  REQUEST/WAIT/INZONE 区間への出入りを判定する
- 各 zone_id に対して WHILL API へ通行ロック(pass-request)を要求し、TTL が切れないように
  ハートビート(再POST)し続ける
- 停止が必要な場面では /rcio/cmd (auto_control) へ停止/解放を発行する。実際の /slow への
  反映は、それを受け取った rcio が今までどおり行う。

このノード(HTTP)を使うか mqtt_bridge_node(MQTT)を使うかは、走行場所ごとに launch で人手で
切り替える想定（同じ mqtt_bridge パッケージに同居しているだけで、両者にコード上の依存関係は無い）。
whill.yaml が存在しない/APIが未設定の場所では、そのまま何もせず待機する。

## Conflict Zone の区間種別（whill.yaml の zone_type）と挙動

- REQUEST : コンフリクトゾーン手前の手前。止まらない。grantedを持とうとし続ける(POST heartbeat)。
- WAIT    : コンフリクトゾーン手前。grantedを持とうとし続けつつ、grantedを持っていなければ実際に停止する。
- INZONE  : コンフリクトゾーン内。止まらない。grantedを持とうとし続ける(脱出優先)。

停止予定(pending stop)は REQUEST/WAIT 区間で有効になり、granted 取得 または INZONE 進入で解除される。
実際に停止指示を出すのは、pending stop が有効 かつ 現在 WAIT 区間にいるときだけ。

## /rcio/cmd への反映

rcio.py の cmd_auto_control が受け付ける形式(factorはWHILL用として'whill'固定)で発行する。
rcio側で slow_check を通して /slow への実際のpublish(factor+"/0/-0.45"等)に変換される。

    {'cmd': 'auto_control', 'action': 'stop',    'factor': 'whill', 'id': '<uuid>'}
    {'cmd': 'auto_control', 'action': 'release', 'factor': 'whill', 'id': '<uuid>'}

## 設定ファイル

- API接続情報: $CONFIG_DIR/whill_api.yaml ( .gitignore 済み。base_url, token )
    base_url: "https://sp.autonomous-mobility.whill.cloud/api/v1"  # 開発環境用エンドポイント
    token: ""     # TODO: robot単位のアクセストークンを設定する

- Conflict Zone定義: $DATA_TEXT_DIR/{map}/{lane}/whill.yaml
  (whill-conflict-zones: [...] 形式。詳細はサンプルのwhill.yamlを参照)
"""

import os
import time
import uuid
from pathlib import Path

import requests
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BypassWhill(Node):

    API_CONFIG_FILENAME = 'whill_api.yaml'  # $CONFIG_DIR/whill_api.yaml
    ZONE_CONFIG_FILENAME = 'whill.yaml'      # $DATA_TEXT_DIR/{map}/{lane}/whill.yaml

    ZONE_TYPES = ('REQUEST', 'WAIT', 'INZONE')

    HTTP_TIMEOUT_SEC = 3.0
    WORKER_INTERVAL_SEC = 0.5

    DEFAULT_TTL_SECONDS = 120.0
    MIN_HEARTBEAT_INTERVAL_SEC = 3.0
    ERROR_RETRY_INTERVAL_SEC = 5.0     # MIMS側(whill_transport)のERROR_RETRY_INTERVALに合わせる
    WAITING_RETRY_DEFAULT_SEC = 5.0    # Retry-Afterヘッダが無かった場合のfallback
    STOP_REPUBLISH_INTERVAL_SEC = 2.0  # 停止指示のロスに備えた再送間隔

    STOP_FACTOR = 'whill'   # rcio.py の cmd_auto_control が受け付けるfactor名
    ACTION_STOP = 'stop'
    ACTION_RELEASE = 'release'

    def __init__(self):
        super().__init__('bypass_whill')

        self.cmd_pub = self.create_publisher(String, '/rcio/cmd', 1)
        self.create_subscription(String, '/vehicle_current', self.cb_vehicle_current, 1)

        self.base_url, self.token = self._load_api_config()
        if not self.base_url or not self.token:
            self.get_logger().warn(
                f'whill API設定({self.API_CONFIG_FILENAME})のbase_url/tokenが未設定です。'
                ' HTTPリクエストは行わず待機します。'
            )
        else:
            # tokenの値そのものはログに出さない
            self.get_logger().info(f'whill API設定を読み込みました: base_url={self.base_url}')

        zone_config_path = self._resolve_zone_config_path()
        self._zones, self._destination_ids = self._load_zone_config(zone_config_path)
        if not self._zones:
            self.get_logger().warn(
                f'whillコンフリクトゾーン設定が見つかりません({zone_config_path})。'
                ' ノードは何もせず待機します。'
            )
        else:
            self.get_logger().info(
                f'whillコンフリクトゾーン設定を読み込みました({zone_config_path}): '
                f'zone={list(self._zones.keys())}'
            )

        # /vehicle_current(waypointId)から求めた、zone_idごとの「今いる区間種別」(REQUEST/WAIT/INZONE/None)
        self._zone_target_type = {zone_id: None for zone_id in self._zones}
        # zone_idごとのAPI保持状態
        self._zone_state = {
            zone_id: {'api_state': None, 'ttl_seconds': None, 'next_heartbeat_time': 0.0}
            for zone_id in self._zones
        }

        self._stop_active = False
        self._last_stop_pub_time = 0.0
        self._last_lanelet_id = None  # ログ表示用

        if self.base_url and self.token and self._destination_ids:
            # 起動時一回だけの疎通確認・キャッシュ取得（同期呼び出し。失敗しても継続動作）
            self._fetch_destinations_and_zones()

        # 以降の周期処理（HTTP通信・/slow反映）はすべてこのタイマーコールバック内で行う
        self.timer = self.create_timer(self.WORKER_INTERVAL_SEC, self._tick)

    # ------------------------------------------------------------------
    # 設定読み込み
    # ------------------------------------------------------------------

    def _load_api_config(self):
        path = os.path.expandvars(os.path.join('$CONFIG_DIR', self.API_CONFIG_FILENAME))
        base_url = ''
        token = ''
        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            base_url = (cfg.get('base_url') or '').rstrip('/')
            token = cfg.get('token') or ''
        except Exception as e:
            self.get_logger().warn(f'whill API設定ファイルの読み込みに失敗しました({path}): {e}')
        return base_url, token

    def _resolve_zone_config_path(self):
        # vehicle_current.py の _resolve_map_data_dir と同じロジックで
        # $DATA_TEXT_DIR/{map}/{lane}/ を求める
        map_car_yaml_path = os.path.expandvars('$CONFIG_DIR/map_car.yaml')
        try:
            with open(map_car_yaml_path, 'r') as f:
                map_car = yaml.safe_load(f)
            map_name = Path(map_car['map_path']).name
            lane_name = Path(Path(map_car['lanelet2_name']).name).stem
        except Exception as e:
            self.get_logger().error(f'{map_car_yaml_path} の読み込みに失敗しました: {e}')
            return ''

        data_dir = os.path.expandvars(f'$DATA_TEXT_DIR/{map_name}/{lane_name}')
        return os.path.join(data_dir, self.ZONE_CONFIG_FILENAME)

    def _load_zone_config(self, path):
        zones = {}          # zone_id -> [(index_from, index_to, zone_type), ...]
        destination_ids = []

        if not path or not os.path.exists(path):
            return zones, destination_ids

        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f'whill.yaml の読み込みに失敗しました({path}): {e}')
            return zones, destination_ids

        for dest in cfg.get('whill-conflict-zones', []) or []:
            destination_id = dest.get('destination_id')
            if destination_id:
                destination_ids.append(destination_id)

            for zone in dest.get('zones', []) or []:
                zone_id = zone.get('zone_id')
                if not zone_id:
                    continue

                items = []
                for item in zone.get('items', []) or []:
                    try:
                        idx_from = int(item['index_from'])
                        idx_to = int(item['index_to'])
                        zone_type = item['zone_type']
                    except (KeyError, TypeError, ValueError):
                        continue
                    if zone_type not in self.ZONE_TYPES:
                        continue
                    items.append((idx_from, idx_to, zone_type))

                zones.setdefault(zone_id, []).extend(items)

        return zones, destination_ids

    def _auth_headers(self):
        return {'Authorization': f'Bearer {self.token}'}

    # ------------------------------------------------------------------
    # /vehicle_current 監視（通信は行わずtargetを更新するだけ）
    # ------------------------------------------------------------------

    def cb_vehicle_current(self, msg):
        # vehicle_current.py と同様、String化されたdict literalとして届く
        try:
            data = eval(msg.data)
            lanelet_id = int(data.get('waypointId'))
        except Exception as e:
            self.get_logger().debug(f'whill /vehicle_current parse失敗（無視）: {e}')
            return  # 未確定・パース不能な内容は無視（直前の状態を維持）

        if lanelet_id < 0:
            return

        changed = False
        for zone_id, items in self._zones.items():
            zone_type = None
            for (idx_from, idx_to, zt) in items:
                if idx_from <= lanelet_id <= idx_to:
                    zone_type = zt
                    break
            if self._zone_target_type.get(zone_id) != zone_type:
                changed = True
            self._zone_target_type[zone_id] = zone_type

        # 区間の出入り(REQUEST/WAIT/INZONE/圏外)が変化した時だけログを出す(毎回だと流量が多いため)
        if changed or lanelet_id != self._last_lanelet_id:
            zones_str = ', '.join(
                f'{zid}:{zt}' for zid, zt in self._zone_target_type.items() if zt is not None
            ) or 'none'
            self.get_logger().info(f'whill lanelet_id={lanelet_id} zones=[{zones_str}]')
        self._last_lanelet_id = lanelet_id

    # ------------------------------------------------------------------
    # 周期処理（タイマーコールバック。HTTP通信＋/rcio/cmd反映）
    # ------------------------------------------------------------------

    def _tick(self):
        # タイマーコールバック内で例外を投げると以降このタイマーが呼ばれなくなるため、
        # ここで必ず捕捉してログに出すだけにする。
        try:
            self._tick_impl()
        except Exception as e:
            self.get_logger().error(f'whill tick failed: {e}')

    def _tick_impl(self):
        now = time.time()
        any_stop_needed = False

        for zone_id in self._zones:
            zone_type = self._zone_target_type.get(zone_id)
            st = self._zone_state[zone_id]

            if zone_type is None:
                # ゾーン系から完全に抜けた -> 保持しているロックを解放する
                if st['api_state'] is not None:
                    self._release_zone(zone_id, st)
                continue

            # REQUEST/WAIT/INZONE いずれの区間でも、grantedを持とうとし続ける
            if st['api_state'] is None or now >= st['next_heartbeat_time']:
                self._request_zone(zone_id, st, now)

            # 停止予定: REQUEST/WAIT区間 かつ granted未取得。
            # granted取得 or INZONE進入で解除。実際に止まるのはWAIT区間のときだけ。
            pending_stop = zone_type in ('REQUEST', 'WAIT') and st['api_state'] != 'granted'
            if zone_type == 'WAIT' and pending_stop:
                any_stop_needed = True

        # 毎tickの全体スナップショット（流量が多いのでdebugのみ）
        self.get_logger().debug(
            f'whill tick: any_stop_needed={any_stop_needed} '
            f'targets={self._zone_target_type} '
            f'states={ {zid: st["api_state"] for zid, st in self._zone_state.items()} }'
        )

        self._update_cmd_pub(any_stop_needed, now)

    def _request_zone(self, zone_id, st, now):
        if not (self.base_url and self.token):
            self._set_zone_error(zone_id, st, now)
            return

        url = f'{self.base_url}/conflict-zones/{zone_id}/pass-requests'
        self.get_logger().info(f'whill POST {url}')
        try:
            resp = requests.post(url, headers=self._auth_headers(), timeout=self.HTTP_TIMEOUT_SEC)
        except requests.RequestException as e:
            self.get_logger().warn(f'whill POST {url} 失敗: {e}')
            self._set_zone_error(zone_id, st, now)
            return
        self.get_logger().info(f'whill POST {url} -> {resp.status_code} {resp.text}')

        try:
            body = resp.json()
        except ValueError:
            body = {}

        if resp.status_code == 200:
            # granted: MIMS側(whill_transport)に合わせ、TTLの2/3経過時点で次のheartbeatを打つ
            new_state = 'granted'
            ttl = body.get('ttl_seconds', self.DEFAULT_TTL_SECONDS)
            st['ttl_seconds'] = ttl
            interval = max((ttl // 3) * 2, self.MIN_HEARTBEAT_INTERVAL_SEC)
        elif resp.status_code == 202:
            # waiting: TTLではなくRetry-Afterヘッダに従って短い間隔で再問い合わせする
            new_state = 'waiting'
            retry_after = resp.headers.get('Retry-After')
            try:
                interval = float(retry_after)
            except (TypeError, ValueError):
                interval = self.WAITING_RETRY_DEFAULT_SEC
            interval = max(interval, self.MIN_HEARTBEAT_INTERVAL_SEC)
        else:
            self.get_logger().warn(
                f'whill pass-request 予期しない応答 zone={zone_id}: {resp.status_code} {resp.text}'
            )
            self._set_zone_error(zone_id, st, now)
            return

        # granted保持中のheartbeat等、状態が変わらない場合も含めて毎回ポーリング結果をログに出す
        changed_mark = '' if new_state == st['api_state'] else f' (was {st["api_state"]})'
        self.get_logger().info(
            f'whill zone={zone_id} poll -> {new_state}{changed_mark} '
            f'(next in {interval:.0f}s, lanelet_id={self._last_lanelet_id})'
        )
        st['api_state'] = new_state
        st['next_heartbeat_time'] = now + interval

    def _set_zone_error(self, zone_id, st, now):
        # MIMS側(whill_transport)に合わせ、失敗時は前回のgranted等を信じ続けず
        # 「保持できていない」ものとして扱う(WAIT中なら停止が再度かかる)。
        if st['api_state'] != 'error':
            self.get_logger().warn(f'whill zone={zone_id} state -> error (lanelet_id={self._last_lanelet_id})')
        else:
            self.get_logger().debug(f'whill zone={zone_id} still error, retrying (lanelet_id={self._last_lanelet_id})')
        st['api_state'] = 'error'
        st['next_heartbeat_time'] = now + self.ERROR_RETRY_INTERVAL_SEC

    def _release_zone(self, zone_id, st):
        if self.base_url and self.token:
            url = f'{self.base_url}/conflict-zones/{zone_id}/pass-requests'
            self.get_logger().info(f'whill DELETE {url}')
            try:
                resp = requests.delete(url, headers=self._auth_headers(), timeout=self.HTTP_TIMEOUT_SEC)
                self.get_logger().info(f'whill DELETE {url} -> {resp.status_code} {resp.text}')
            except requests.RequestException as e:
                # 失敗してもTTL切れで自動解放されるため、ローカル状態は解放しておく
                self.get_logger().warn(f'whill DELETE {url} 失敗: {e}')

        self.get_logger().info(f'whill zone={zone_id} state -> released (lanelet_id={self._last_lanelet_id})')
        st['api_state'] = None
        st['ttl_seconds'] = None
        st['next_heartbeat_time'] = 0.0

    def _update_cmd_pub(self, any_stop_needed, now):
        if any_stop_needed and not self._stop_active:
            self._publish_rcio_cmd(self.ACTION_STOP)
            self._stop_active = True
            self._last_stop_pub_time = now
        elif not any_stop_needed and self._stop_active:
            self._publish_rcio_cmd(self.ACTION_RELEASE)
            self._stop_active = False
        elif any_stop_needed and now - self._last_stop_pub_time >= self.STOP_REPUBLISH_INTERVAL_SEC:
            # メッセージロス対策の再送
            self._publish_rcio_cmd(self.ACTION_STOP)
            self._last_stop_pub_time = now

    def _publish_rcio_cmd(self, action):
        # rcio.py の cb_cmd -> cmd_auto_control が読む形式(yaml_ut.load = yaml.safe_load でパースされる)
        cmd = {
            'cmd': 'auto_control',
            'action': action,
            'factor': self.STOP_FACTOR,
            'id': str(uuid.uuid4()),
        }
        self.get_logger().info(f'whill /rcio/cmd publish: {cmd} (lanelet_id={self._last_lanelet_id})')
        self.cmd_pub.publish(String(data=str(cmd)))

    # ------------------------------------------------------------------
    # 起動時の destination/zone 一覧取得（キャッシュ・疎通確認目的。失敗しても致命的ではない）
    # ------------------------------------------------------------------

    def _fetch_destinations_and_zones(self):
        url = f'{self.base_url}/destinations'
        self.get_logger().info(f'whill GET {url}')
        try:
            resp = requests.get(url, headers=self._auth_headers(), timeout=self.HTTP_TIMEOUT_SEC)
            self.get_logger().info(f'whill GET {url} -> {resp.status_code} {resp.text}')
            resp.raise_for_status()
        except Exception as e:
            self.get_logger().warn(f'whill GET {url} 失敗: {e}')

        for destination_id in self._destination_ids:
            url = f'{self.base_url}/destinations/{destination_id}/conflict-zones'
            self.get_logger().info(f'whill GET {url}')
            try:
                resp = requests.get(url, headers=self._auth_headers(), timeout=self.HTTP_TIMEOUT_SEC)
                self.get_logger().info(f'whill GET {url} -> {resp.status_code} {resp.text}')
                resp.raise_for_status()
            except Exception as e:
                self.get_logger().warn(f'whill GET {url} 失敗: {e}')

    # ------------------------------------------------------------------

    def destroy_node(self):
        self.timer.cancel()

        # ベストエフォートで保持中のロックを解放してから終了する
        for zone_id, st in list(self._zone_state.items()):
            if st['api_state'] is not None:
                try:
                    self._release_zone(zone_id, st)
                except Exception:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BypassWhill()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
