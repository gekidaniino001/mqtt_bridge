#!/bin/bash
# 羽田 Haneda-Lab-WIFI 向けのローミング設定を恒久化する / 元に戻すスクリプト
#
# 仕組み:
#   NetworkManager の dispatcher スクリプトを置き、Haneda-Lab-WIFI に接続(up)する度に
#   wpa_supplicant へ bgscan と scan_freq を設定し直す。
#   (NM は再接続・スリープ復帰の度に bgscan を既定値 simple:30:-70:86400 に戻すため)
#   ローミング(AP切替)では NM の再接続は起きないので設定は維持される。
#   あわせてデフォルトゲートウェイを切り替える:
#     install   : Haneda-Lab-WIFI を唯一のデフォゲにする(有線/LTE等の他の接続は never-default にする。元の値は退避)
#     uninstall : Haneda-Lab-WIFI をデフォゲから外し、他の接続は退避しておいた元の値に戻す
#
# 使い方 (すべて sudo で実行):
#   sudo ./haneda_roaming.sh install      # dispatcher を設置し、デフォゲ切替も行い、今の接続にも即反映
#   sudo ./haneda_roaming.sh status       # 設置状況・デフォゲ設定・wpa_supplicant の現在値を表示
#   sudo ./haneda_roaming.sh apply        # 今の接続にだけ bgscan/scan_freq を反映(設置・デフォゲ切替はしない)
#   sudo ./haneda_roaming.sh uninstall    # dispatcher を削除・デフォゲを戻し、Wi-Fi を再接続して NM 既定値に戻す
#   sudo ./haneda_roaming.sh uninstall --no-reconnect   # 再接続せずに戻す(瞬断を避けたい時)

set -euo pipefail

# ---- 設定値 (2026-09-24 羽田T1北サテライト 8周目の設定) ----
# Wi-Fiのインターフェース名。PCごとに番号が違うので、空なら自動検出(最初に見つかったWi-Fiデバイス)
IFACE=""
# 対象のSSID。接続プロファイル名はPCごとに違う(例: "Haneda-Lab-WIFI 1")ので、プロファイルはSSIDで探す
SSID="Haneda-Lab-WIFI"
if [ -z "$IFACE" ]; then
    IFACE=$(nmcli -g DEVICE,TYPE device | sed -n 's/:wifi$//p' | head -1)
    if [ -z "$IFACE" ]; then
        echo "Wi-Fiデバイスが見つかりません。スクリプト冒頭の IFACE に手で設定してください" >&2
        exit 1
    fi
fi
BGSCAN="simple:4:-65:8"
# 2026-09-24 に観測した Haneda-Lab-WIFI の全周波数。AP側のチャンネル変更(DFS等)があれば更新が必要
SCAN_FREQ="2412 2437 2462 5180 5260 5300 5320 5500 5540 5580 5660 5700"
NM_DEFAULT_BGSCAN="simple:30:-70:86400"
HOOK="/etc/NetworkManager/dispatcher.d/90-haneda-roaming"
LOG_TAG="haneda-roaming"
# install 前の他接続の never-default 値の退避先(uninstall で元に戻すのに使う)
STATE_DIR="/var/lib/haneda-roaming"
ROUTE_BACKUP="$STATE_DIR/never-default.backup"
# デフォゲを持ちうる接続の種類。docker等のbridgeやVPNは対象外
GW_TYPES="802-3-ethernet gsm cdma pppoe bluetooth bond vlan team"

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "sudo で実行してください: sudo $0 $*" >&2
        exit 1
    fi
}

# dispatcher 本体。install 時に設定値を埋め込んで $HOOK に書き出す
write_hook() {
    cat > "$HOOK" <<EOF
#!/bin/bash
# haneda_roaming.sh が生成 ($(date '+%Y-%m-%d %H:%M:%S'))。削除は haneda_roaming.sh uninstall で行う
IFACE="$IFACE"
SSID="$SSID"
BGSCAN="$BGSCAN"
SCAN_FREQ="$SCAN_FREQ"
LOG_TAG="$LOG_TAG"

[ "\$1" = "\$IFACE" ] || exit 0
case "\$2" in up|reapply) ;; *) exit 0 ;; esac

# wpa_supplicant 側のネットワークIDとSSIDを取得(接続完了直後に取れない場合に備えて最大10秒待つ)
# プロファイル名はPCごとに違うので、対象かどうかはSSIDで判定する
id=""
for _ in \$(seq 1 20); do
    st=\$(wpa_cli -i "\$IFACE" status 2>/dev/null)
    id=\$(echo "\$st" | sed -n 's/^id=//p')
    [ -n "\$id" ] && break
    sleep 0.5
done
if [ -z "\$id" ]; then
    logger -t "\$LOG_TAG" "network id not found on \$IFACE; skip"
    exit 0
fi
[ "\$(echo "\$st" | sed -n 's/^ssid=//p')" = "\$SSID" ] || exit 0

wpa_cli -i "\$IFACE" set_network "\$id" scan_freq "\$SCAN_FREQ" >/dev/null
wpa_cli -i "\$IFACE" set_network "\$id" bgscan "\"\$BGSCAN\"" >/dev/null
logger -t "\$LOG_TAG" "applied on \$IFACE id=\$id bgscan=\$BGSCAN scan_freq=\$SCAN_FREQ"
EOF
    chown root:root "$HOOK"
    chmod 755 "$HOOK"
}

# SSID が Haneda-Lab-WIFI の接続プロファイルのUUID一覧(同じSSIDのプロファイルが複数あれば全部)
ssid_profiles() {
    local uuid type
    nmcli -g UUID,TYPE con show | while IFS=: read -r uuid type; do
        [ "$type" = "802-11-wireless" ] || continue
        [ "$(nmcli -g 802-11-wireless.ssid con show "$uuid")" = "$SSID" ] && echo "$uuid"
    done
}

# 一度も接続していないPCにはプロファイルが無い
has_profile() {
    [ -n "$(ssid_profiles)" ]
}

profile_name() {
    nmcli -g connection.id con show "$1"
}

current_id() {
    wpa_cli -i "$IFACE" status 2>/dev/null | sed -n 's/^id=//p'
}

current_ssid() {
    wpa_cli -i "$IFACE" status 2>/dev/null | sed -n 's/^ssid=//p'
}

# 今の接続へ即反映する。同じ値の再設定では bgscan が再初期化されないため、
# 一度 long interval を +1 した値を入れてから本来の値に戻して、確実に再初期化させる
apply_now() {
    local id ssid tmp_bgscan
    id=$(current_id)
    ssid=$(current_ssid)
    if [ -z "$id" ] || [ "$ssid" != "$SSID" ]; then
        echo "今は $SSID に接続していないため、即時反映はスキップしました (ssid='${ssid}')"
        return 0
    fi
    tmp_bgscan="${BGSCAN%:*}:$(( ${BGSCAN##*:} + 1 ))"
    wpa_cli -i "$IFACE" set_network "$id" scan_freq "$SCAN_FREQ" >/dev/null
    wpa_cli -i "$IFACE" set_network "$id" bgscan "\"$tmp_bgscan\"" >/dev/null
    wpa_cli -i "$IFACE" set_network "$id" bgscan "\"$BGSCAN\"" >/dev/null
    logger -t "$LOG_TAG" "applied manually on $IFACE id=$id bgscan=$BGSCAN scan_freq=$SCAN_FREQ"
    echo "反映しました: id=$id bgscan=$BGSCAN"
    echo "              scan_freq=$SCAN_FREQ"
}

# Wi-Fi以外でデフォゲを持ちうる接続プロファイルのUUID一覧
other_gw_profiles() {
    local uuid type
    nmcli -g UUID,TYPE con show | while IFS=: read -r uuid type; do
        case " $GW_TYPES " in *" $type "*) echo "$uuid" ;; esac
    done
}

# 接続中なら設定変更を即反映する(未接続なら次回接続時に反映される)
reapply_profile() {
    local dev
    dev=$(nmcli -g GENERAL.DEVICES con show "$1" 2>/dev/null || true)
    [ -n "$dev" ] && nmcli device reapply "$dev" >/dev/null || true
}

route_install() {
    local uuid
    mkdir -p "$STATE_DIR"
    # 2回目以降の install で退避値を上書きしないよう、退避は初回のみ
    if [ ! -f "$ROUTE_BACKUP" ]; then
        for uuid in $(other_gw_profiles); do
            echo "$uuid $(nmcli -g ipv4.never-default con show "$uuid") $(nmcli -g ipv6.never-default con show "$uuid")"
        done > "$ROUTE_BACKUP"
    fi
    # 値が変わる時だけ modify/reapply して、接続中の回線への影響を最小にする
    for uuid in $(other_gw_profiles); do
        if [ "$(nmcli -g ipv4.never-default con show "$uuid")" != "yes" ]; then
            nmcli con modify "$uuid" ipv4.never-default yes ipv6.never-default yes
            reapply_profile "$uuid"
        fi
        echo "デフォゲ対象外にしました: $(nmcli -g connection.id con show "$uuid")"
    done
    for uuid in $(ssid_profiles); do
        if [ "$(nmcli -g ipv4.never-default con show "$uuid")" != "no" ]; then
            nmcli con modify "$uuid" ipv4.never-default no ipv6.never-default no
            reapply_profile "$uuid"
        fi
        echo "デフォゲにしました: $(profile_name "$uuid")"
    done
    echo "$SSID を唯一のデフォゲにしました"
    logger -t "$LOG_TAG" "default gateway: $SSID only"
}

route_uninstall() {
    local uuid v4 v6
    # プロファイルが無くても、他の接続の復元は必ず行う
    if has_profile; then
        for uuid in $(ssid_profiles); do
            nmcli con modify "$uuid" ipv4.never-default yes ipv6.never-default yes
            echo "デフォゲから外しました: $(profile_name "$uuid")"
        done
    else
        echo "$SSID の接続プロファイルが無いため、Wi-Fi側の設定変更はスキップしました"
    fi
    if [ -f "$ROUTE_BACKUP" ]; then
        while read -r uuid v4 v6; do
            # 退避後に削除されたプロファイルは飛ばす
            nmcli -g UUID con show "$uuid" >/dev/null 2>&1 || continue
            nmcli con modify "$uuid" ipv4.never-default "$v4" ipv6.never-default "$v6"
            reapply_profile "$uuid"
            echo "元に戻しました: $(nmcli -g connection.id con show "$uuid") (ipv4.never-default=$v4)"
        done < "$ROUTE_BACKUP"
        rm -f "$ROUTE_BACKUP"
    else
        echo "退避ファイルが無いため、他の接続のデフォゲ設定は変更していません"
    fi
    logger -t "$LOG_TAG" "default gateway: $SSID removed, others restored"
}

show_status() {
    local id uuid
    echo "デフォゲ   : $(ip route show default | tr '\n' ' ')"
    if has_profile; then
        for uuid in $(ssid_profiles); do
            echo "             $(profile_name "$uuid") ipv4.never-default=$(nmcli -g ipv4.never-default con show "$uuid")"
        done
    else
        echo "             $SSID の接続プロファイルなし"
    fi
    for uuid in $(other_gw_profiles); do
        echo "             $(nmcli -g connection.id con show "$uuid") ipv4.never-default=$(nmcli -g ipv4.never-default con show "$uuid")"
    done
    if [ -f "$HOOK" ]; then
        echo "dispatcher : 設置済み ($HOOK)"
        grep -E '^(BGSCAN|SCAN_FREQ)=' "$HOOK" | sed 's/^/             /'
    else
        echo "dispatcher : 未設置"
    fi
    id=$(current_id)
    echo "接続中SSID : $(current_ssid) (id=${id:-なし})"
    if [ -n "$id" ]; then
        echo "bgscan     : $(wpa_cli -i "$IFACE" get_network "$id" bgscan)"
        echo "scan_freq  : $(wpa_cli -i "$IFACE" get_network "$id" scan_freq)"
    fi
    if pgrep -x gnome-control-center >/dev/null; then
        echo "注意       : 設定画面(gnome-control-center)が起動中。bgscan のスキャン結果が横取りされるので閉じてください"
    fi
}

do_uninstall() {
    local reconnect=1 id
    [ "${1:-}" = "--no-reconnect" ] && reconnect=0
    if [ -f "$HOOK" ]; then
        rm -f "$HOOK"
        echo "dispatcher を削除しました ($HOOK)"
    else
        echo "dispatcher は設置されていませんでした"
    fi
    route_uninstall
    if [ "$(current_ssid)" != "$SSID" ]; then
        echo "今は $SSID に接続していないため、次回接続時から NM 既定値になります"
        warn_if_no_default
        logger -t "$LOG_TAG" "uninstalled (not connected)"
        return 0
    fi
    if [ "$reconnect" -eq 1 ]; then
        # 再接続すると NM が wpa_supplicant のネットワーク設定を作り直すので、確実に既定値へ戻る(数秒の瞬断あり)
        # デフォゲから外す設定もこの再接続で反映される
        echo "Wi-Fi を再接続して NM 既定値に戻します(数秒切れます)..."
        # 今つながっているプロファイル(名前はPCごとに違う)を張り直す
        nmcli con up "$(nmcli -g GENERAL.CONNECTION device show "$IFACE")" >/dev/null
        echo "再接続しました"
    else
        # デフォゲから外す設定を再接続なしで反映
        nmcli device reapply "$IFACE" >/dev/null || true
        id=$(current_id)
        wpa_cli -i "$IFACE" set_network "$id" bgscan "\"$NM_DEFAULT_BGSCAN\"" >/dev/null
        # scan_freq を空にして全チャンネルスキャンに戻す。失敗した場合は次回の再接続で戻る
        if ! wpa_cli -i "$IFACE" set_network "$id" scan_freq "" | grep -q OK; then
            echo "scan_freq の解除に失敗しました。次回の再接続で元に戻ります"
        fi
        echo "再接続せずに bgscan=$NM_DEFAULT_BGSCAN に戻しました"
    fi
    warn_if_no_default
    logger -t "$LOG_TAG" "uninstalled (reconnect=$reconnect)"
}

warn_if_no_default() {
    if [ -z "$(ip route show default)" ]; then
        echo "注意: 現在デフォルトゲートウェイがありません(Wi-Fi以外の回線が未接続の可能性)。インターネットに出られない状態です"
    fi
}

case "${1:-}" in
    install)
        require_root "$@"
        # プロファイルが無いと途中で失敗し、他の接続だけデフォゲから外れた状態になるので、何も変更せずに止める
        if ! has_profile; then
            echo "$SSID の接続プロファイルがありません。先に一度 $SSID に接続してから install してください" >&2
            exit 1
        fi
        route_install
        write_hook
        echo "dispatcher を設置しました ($HOOK)"
        apply_now
        ;;
    apply)
        require_root "$@"
        apply_now
        ;;
    status)
        require_root "$@"
        show_status
        ;;
    uninstall)
        require_root "$@"
        do_uninstall "${2:-}"
        ;;
    *)
        sed -n '2,19p' "$0"
        exit 1
        ;;
esac
