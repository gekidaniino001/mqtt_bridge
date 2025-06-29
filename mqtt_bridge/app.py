import inject
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node

from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object
from std_msgs.msg import String
import dbg,threading,time,datetime


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
        self.prev_reconnect = -1

    def cb_hb_mims(self, msg):
        # payload = eval(msg.data)
        # s_format = '%Y-%m-%d %H:%M:%S.%f'
        # dt = datetime.datetime.strptime(payload["timestamp"], s_format)
        # self.prev_hb = dt
        self.prev_hb = datetime.datetime.fromtimestamp(time.time())

    def timer_cb(self):
        global mqtt_client
        if self.prev_hb:
            if (datetime.datetime.fromtimestamp(time.time()) - self.prev_hb).seconds < 5:
                self.get_logger().info("---OK---")
                # pass
            elif ( time.time() - self.prev_reconnect ) >= 15:
                self.get_logger().warn("Reconnecting MQTT...")
                self.get_logger().warn(f"last mims_hb is {(datetime.datetime.fromtimestamp(time.time()) - self.prev_hb)} ago")
                self.reset_bridges('mqtt_to_ros')
                try:
                    if mqtt_client.is_connected():
                        mqtt_client.disconnect()
                except Exception as e:
                    self.get_logger().warn(f"Disconnect error: {e}")

                try:
                    mqtt_client._thread_terminate = True
                    mqtt_client.loop_stop()
                except Exception as e:
                    self.get_logger().warn(f"Loop stop error: {e}")

                # v1.5.1対応：threadがまだ動いていたらjoinする
                thread = getattr(mqtt_client, "_thread", None)
                if thread and thread.is_alive():
                    self.get_logger().warn("Joining MQTT thread manually (paho-mqtt 1.5.x fallback)")
                    try:
                        thread.join()
                    except Exception as e:
                        self.get_logger().warn(f"Join failed: {e}")

                mqtt_client = None

                # MQTT再初期化（再接続）
                mqtt_bridge_node(spin=False)
                self.prev_reconnect = time.time()

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

    mqtt_private_path = mqtt_node.get_parameter("mqtt.private_path").value

    # create mqtt client
    mqtt_client_factory_name = mqtt_node.get_parameter_or(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory"
    )

    global mqtt_client
    # mqtt_client.default_mqtt_client_factory
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)
    mqtt_client.reconnect_delay_set(min_delay=0, max_delay=0)

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
    while not connect_flg:
        try:
            mqtt_client.connect(**conn_params)
            connect_flg = True
        except:
            mqtt_node.get_logger().info("wait connect...")
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
