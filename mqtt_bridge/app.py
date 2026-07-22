import inject
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node

from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object
from std_msgs.msg import String
import dbg,threading,time,datetime,traceback
import os,socket,struct,subprocess


def create_config(mqtt_client, serializer, deserializer, mqtt_private_path):
    if isinstance(serializer.value, str):
        serializer = lookup_object(serializer.value)
    if isinstance(deserializer.value, str):
        deserializer = lookup_object(deserializer.value)
    private_path_extractor = create_private_path_extractor(mqtt_private_path)

    def config(binder):
        binder.bind("serializer", serializer)
        binder.bind("deserializer", deserializer)
        binder.bind(mqtt.Client, mqtt_client)
        binder.bind("mqtt_private_path_extractor", private_path_extractor)

    return config


class MqttNode(Node):
    def __init__(self):
        super().__init__("mqtt_bridge_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,)
        self.prev_hb = False
        self.mims_hb_sub = self.create_subscription(String, "/hb_mims", self.cb_hb_mims, 1)
        timer_period = 3.00  # 秒
        self.bridges = {
            'mqtt_to_ros': [],
            'ros_to_mqtt': [],
        }
        self.timer = self.create_timer(timer_period, self.timer_cb)  # 指定間隔でcbを呼び出す
        self._timer_period = timer_period
        self._last_timer_ts = None
        self._last_timer_delay = 0.0
        self.broker_host = None
        self.broker_port = None
        self.last_connected_ts = None
        # 新規接続確立後、最低この秒数は再接続をトリガーしない
        # (ハンドシェイク〜最初のhb受信が終わる前にreset_bridgesが走るスラッシングを防ぐ)
        self.RECONNECT_SETTLE_SEC = 20.0

    def cb_hb_mims(self, msg):
        # payload = eval(msg.data)
        # s_format = '%Y-%m-%d %H:%M:%S.%f'
        # dt = datetime.datetime.strptime(payload["timestamp"], s_format)
        # self.prev_hb = dt
        self.prev_hb = datetime.datetime.fromtimestamp(time.time())

    def _get_default_gateway(self):
        """デフォルトゲートウェイのIPを/proc/net/routeから取得する（ローカル無線/回線の生死確認用）。
        デフォルトルートが複数存在する場合（例: FS040U/050Wの2経路）、行の並び順はmetric順とは限らないため、
        metricが最小のルートを実際のデフォルトゲートウェイとして選ぶ。"""
        try:
            candidates = []
            with open("/proc/net/route") as f:
                for line in f.readlines()[1:]:
                    fields = line.strip().split()
                    if len(fields) < 8:
                        continue
                    if fields[1] == "00000000":
                        metric = int(fields[6])
                        gw = socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
                        candidates.append((metric, gw))
            if candidates:
                return min(candidates, key=lambda x: x[0])[1]
        except Exception as e:
            self.get_logger().warn(f"[MQTT diag] failed to read default gateway: {e}")
        return None

    def _ping(self, host, timeout=1):
        if not host:
            return None
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", str(timeout), host],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout + 1,
            )
            return result.returncode == 0
        except Exception as e:
            self.get_logger().warn(f"[MQTT diag] ping to {host} failed: {e}")
            return False

    def _check_broker_tcp(self, host, port, timeout=2):
        """TCP接続確認。DNS解決失敗(自分側のリゾルバ/経路の問題)とTCP接続失敗
        (ブローカー側/ファイアウォールの可能性)を区別してログに残す。"""
        if not host or not port:
            return None
        try:
            addr = socket.gethostbyname(host)
        except Exception as e:
            self.get_logger().warn(f"[MQTT diag] DNS resolve failed for {host}: {e}")
            return False
        try:
            with socket.create_connection((addr, int(port)), timeout=timeout):
                return True
        except Exception as e:
            self.get_logger().warn(f"[MQTT diag] tcp connect to {addr}:{port} failed: {e}")
            return False

    def _diagnose_disconnect(self):
        """切断検知時に、電波(ローカル回線)/ネットワーク(先方)/マシン(処理遅延)のどこに問題がありそうかを切り分けてログに残す"""
        gateway = self._get_default_gateway()
        gateway_ok = self._ping(gateway)
        broker_ok = self._check_broker_tcp(self.broker_host, self.broker_port)
        try:
            load1, load5, load15 = os.getloadavg()
        except Exception:
            load1 = load5 = load15 = -1.0
        cpu_count = os.cpu_count() or 1

        self.get_logger().warn(
            f"[MQTT diag] gateway={gateway} gateway_ok={gateway_ok} "
            f"broker={self.broker_host}:{self.broker_port} broker_tcp_ok={broker_ok} "
            f"load1={load1:.2f} load5={load5:.2f} cpu_count={cpu_count} "
            f"timer_delay={self._last_timer_delay:.2f}s"
        )

        if gateway_ok is False:
            verdict = "電波(ローカル回線)不良の可能性"
        elif broker_ok is False:
            verdict = "ネットワーク(先方/回線)不良の可能性"
        elif self._last_timer_delay > 1.0 or load1 > cpu_count:
            verdict = "マシン(処理遅延/高負荷)の可能性"
        else:
            verdict = "原因不明(ブローカー/セッション側の可能性)"
        self.get_logger().warn(f"[MQTT diag] verdict: {verdict}")

    def _reconnect_allowed(self, now):
        """新規接続確立後、最低RECONNECT_SETTLE_SEC秒は再接続をトリガーしない。"""
        if self.last_connected_ts is None:
            return True
        return (now - self.last_connected_ts) >= self.RECONNECT_SETTLE_SEC

    def timer_cb(self):
        global mqtt_client
        now = time.time()
        if self._last_timer_ts is not None:
            self._last_timer_delay = now - self._last_timer_ts - self._timer_period
        self._last_timer_ts = now

        if self.prev_hb:
            if (datetime.datetime.fromtimestamp(time.time()) - self.prev_hb).seconds < 5:
                self.get_logger().info("---OK---")
                # pass
            elif self._reconnect_allowed(now):
                self.get_logger().warn("Reconnecting MQTT...")
                self.get_logger().warn(f"last mims_hb is {(datetime.datetime.fromtimestamp(time.time()) - self.prev_hb)} ago")
                threading.Thread(target=self._diagnose_disconnect, daemon=True).start()

                t0 = time.time()
                self.get_logger().warn("[MQTT reconnect] reset_bridges start")
                self.reset_bridges('mqtt_to_ros')
                self.get_logger().warn(f"[MQTT reconnect] reset_bridges done ({time.time() - t0:.2f}s)")

                t0 = time.time()
                try:
                    if mqtt_client.is_connected():
                        mqtt_client.disconnect()
                except Exception as e:
                    self.get_logger().warn(f"Disconnect error: {e}")
                self.get_logger().warn(f"[MQTT reconnect] disconnect() done ({time.time() - t0:.2f}s)")

                t0 = time.time()
                try:
                    mqtt_client._thread_terminate = True
                    mqtt_client.loop_stop()
                except Exception as e:
                    self.get_logger().warn(f"Loop stop error: {e}")
                self.get_logger().warn(f"[MQTT reconnect] loop_stop() done ({time.time() - t0:.2f}s)")

                # v1.5.1対応：threadがまだ動いていたらjoinする
                thread = getattr(mqtt_client, "_thread", None)
                if thread and thread.is_alive():
                    self.get_logger().warn("Joining MQTT thread manually (paho-mqtt 1.5.x fallback)")
                    t0 = time.time()
                    try:
                        thread.join()
                    except Exception as e:
                        self.get_logger().warn(f"Join failed: {e}")
                    self.get_logger().warn(f"[MQTT reconnect] thread.join() done ({time.time() - t0:.2f}s)")

                mqtt_client = None

                # MQTT再初期化（再接続）
                t0 = time.time()
                self.get_logger().warn("[MQTT reconnect] mqtt_bridge_node(spin=False) start")
                try:
                    mqtt_bridge_node(spin=False)
                    self.get_logger().warn(f"[MQTT reconnect] mqtt_bridge_node(spin=False) done ({time.time() - t0:.2f}s)")
                except Exception as e:
                    self.get_logger().error(
                        f"[MQTT reconnect] mqtt_bridge_node(spin=False) failed after {time.time() - t0:.2f}s: {e}\n"
                        f"{traceback.format_exc()}"
                    )
            else:
                self.get_logger().info("---ELSE---")
                

    def add_bridge(self, bridge, mqtt_to_ros=True):
        if mqtt_to_ros:
            self.bridges['mqtt_to_ros'].append(bridge)
        else:
            self.bridges['ros_to_mqtt'].append(bridge)

    def get_bridges(self):
        return self.bridges

    def reset_bridges(self, key='mqtt_to_ros'):
        """ブリッジをリセットする。"""
        if key not in self.bridges.keys():
            self.get_logger().warn(f'unknown bridge key: {key}')
        for brdg in self.bridges[key]:
            brdg.cleanup()
        self.bridges[key] = []

def mqtt_bridge_node(spin=True):
    """_summary_
    mqtt_bridge_nodeを生成する。

    """

    global mqtt_node
    
    if spin:
        mqtt_node = MqttNode()

    # load bridge parameters
    bridge_dict_keys = ["factory", "msg_type", "topic_from", "topic_to"]
    bridge_params = []  # 各topicの変換の為のconfig
    total_bridges = mqtt_node.get_parameter("n_bridges").value  # 変換するtopicの数

    for i in range(total_bridges):
        bridge_n = str((i % total_bridges) + 1)

        bridge_param = mqtt_node.get_parameter(
            "bridge.bridge" + bridge_n
        ).value  # ["mqtt_bridge.bridge:RosToMqttBridge","std_msgs.msg:Bool","/ping","ping"]

        bridge_params.append(dict(zip(bridge_dict_keys, bridge_param)))

    mqtt_params = {
        "client": mqtt_node.get_parameters_by_prefix("mqtt.client"),
        "tls": mqtt_node.get_parameters_by_prefix("mqtt.tls"),
        "account": mqtt_node.get_parameters_by_prefix("mqtt.account"),
        "userdata": mqtt_node.get_parameters_by_prefix("mqtt.userdata"),
        "message": mqtt_node.get_parameters_by_prefix("mqtt.message"),
        "will": mqtt_node.get_parameters_by_prefix("mqtt.will"),
    }

    conn_params = mqtt_node.get_parameters_by_prefix("mqtt.connection")
    # connection:
    # host: a4vg4r4w8fz62-ats.iot.ap-northeast-1.amazonaws.com
    # keepalive: 60
    # port: 8883
    mqtt_node.get_logger().info("------------------------")
    mqtt_node.get_logger().info(str(mqtt_params))
    mqtt_node.get_logger().info("------------------------")
    mqtt_node.get_logger().info(str(conn_params["host"]))

    for key in conn_params.keys():
        conn_params.update({key: conn_params[key].value})

    mqtt_node.broker_host = conn_params.get("host")
    mqtt_node.broker_port = conn_params.get("port")

    mqtt_private_path = mqtt_node.get_parameter("mqtt.private_path").value

    # create mqtt client
    mqtt_client_factory_name = mqtt_node.get_parameter_or(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory"
    )

    global mqtt_client
    # mqtt_client.default_mqtt_client_factory
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)
    mqtt_client.reconnect_delay_set(min_delay=60, max_delay=60)

    # load serializer and deserializer
    serializer = mqtt_node.get_parameter_or("serializer", "msgpack:dumps")
    deserializer = mqtt_node.get_parameter_or("deserializer", "msgpack:loads")

    # dependency injection
    config = create_config(mqtt_client, serializer, deserializer, mqtt_private_path)
    if not spin:
         inject.clear()
    inject.configure(config)

    # configure and connect to MQTT broker
    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect

    connect_flg = False
    attempt = 0
    while not connect_flg:
        attempt += 1
        t0 = time.time()
        try:
            mqtt_client.connect(**conn_params)
            connect_flg = True
            mqtt_node.get_logger().info(
                f"[MQTT reconnect] connect() attempt={attempt} succeeded ({time.time() - t0:.2f}s)"
            )
        except Exception as e:
            mqtt_node.get_logger().info(
                f"wait connect... (attempt={attempt}, {time.time() - t0:.2f}s, err={e})"
            )
            time.sleep(1)

    time.sleep(1)
    for bridge_args in bridge_params:
        ros_to_mqtt = (bridge_args["factory"] == "mqtt_bridge.bridge:RosToMqttBridge")
        if not spin and ros_to_mqtt:
            continue
        # mqtt_node.get_logger().info(str(bridge_args))
        mqtt_node.add_bridge(create_bridge(**bridge_args, ros_node=mqtt_node), not ros_to_mqtt)

    # start MQTT loop
    mqtt_node.get_logger().info(str(mqtt_client._sock))
    mqtt_client.loop_start()

    if spin:
        try:
            rclpy.spin(mqtt_node)
        except KeyboardInterrupt:
            mqtt_node.get_logger().info("Ctrl-C detected")
            mqtt_client.disconnect()
            mqtt_client.loop_stop()

        mqtt_node.destroy_node()


def _on_connect(client, userdata, flags, response_code):
    mqtt_node.last_connected_ts = time.time()
    mqtt_node.get_logger().info("MQTT connected!")
    # mqtt_node.get_logger().info(str(client._sock))
    # mqtt_node.get_logger().info(str(userdata))
    # mqtt_node.get_logger().info(str(flags))
    # mqtt_node.get_logger().info(str(response_code))


def _on_disconnect(client, userdata, response_code):
    mqtt_node.get_logger().warn(f"MQTT disconnected! code={response_code}")
    pass 
    # mqtt_node.get_logger().info("MQTT disconnected")
    # mqtt_node.get_logger().info("retry...")

    # # 切断（既に切れててもOK）
    # try:
    #     if client.is_connected():
    #         client.disconnect()
    # except Exception as e:
    #     mqtt_node.get_logger().warn(f"Disconnect error: {e}")

    # # MQTTループ停止（v1.5では join() されない）
    # try:
    #     mqtt_client._thread_terminate = True
    #     client.loop_stop()
    # except Exception as e:
    #     mqtt_node.get_logger().warn(f"Loop stop error: {e}")

    # # 明示的に join() を fallback として入れる（v1.5対策）
    # thread = getattr(client, "_thread", None)
    # if thread and thread.is_alive():
    #     mqtt_node.get_logger().warn("Joining MQTT thread manually (paho-mqtt 1.5.x fallback)")
    #     try:
    #         thread.join()
    #     except Exception as e:
    #         mqtt_node.get_logger().warn(f"Join failed: {e}")

    # client = None
    # inject.clear()
    # mqtt_node.destroy_node()
    # mqtt_bridge_node(spin=False)

__all__ = ["mqtt_bridge_node"]


# <ssl.SSLSocket fd=10, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 36807), raddr=('54.65.4.57', 8883)>

# <ssl.SSLSocket fd=10, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 58523), raddr=('3.113.90.235', 8883)>
# <ssl.SSLSocket fd=11, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 59419), raddr=('35.73.203.233', 8883)>
