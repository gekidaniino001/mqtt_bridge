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
        self.timer = self.create_timer(timer_period, self.timer_cb)  # 指定間隔でcbを呼び出す

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
            else:
                self.get_logger().info("NG..")
                self.prev_hb = False
                mqtt_client.disconnect()
                mqtt_client.loop_stop()
                mqtt_client._thread_terminate = True
                if threading.current_thread() != mqtt_client._thread:
                    mqtt_client._thread.join()
                    mqtt_client._thread = None
                mqtt_client = None
                # inject.clear()
                # mqtt_node.destroy_node()
                mqtt_bridge_node(spin=False)

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
    global bridges
    bridges = []
    for bridge_args in bridge_params:
        if not spin and bridge_args["factory"] == "mqtt_bridge.bridge:RosToMqttBridge":
            continue
        # mqtt_node.get_logger().info(str(bridge_args))
        bridges.append(create_bridge(**bridge_args, ros_node=mqtt_node))

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
    mqtt_node.get_logger().info("MQTT disconnected")
    mqtt_node.get_logger().info("retry...")
    client.disconnect() 
    client.loop_stop()
    client._thread_terminate = True
    if threading.current_thread() != client._thread:
        client._thread.join()
        client._thread = None
    client = None
    # inject.clear()
    # mqtt_node.destroy_node()
    mqtt_bridge_node(spin=False)

__all__ = ["mqtt_bridge_node"]


# <ssl.SSLSocket fd=10, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 36807), raddr=('54.65.4.57', 8883)>

# <ssl.SSLSocket fd=10, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 58523), raddr=('3.113.90.235', 8883)>
# <ssl.SSLSocket fd=11, family=AddressFamily.AF_INET, type=SocketKind.SOCK_STREAM, proto=6, laddr=('192.168.11.160', 59419), raddr=('35.73.203.233', 8883)>
