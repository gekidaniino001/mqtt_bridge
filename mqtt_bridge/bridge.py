from abc import ABCMeta
from typing import Optional, Type, Dict, Union

import inject
import paho.mqtt.client as mqtt

from .util import lookup_object, extract_values, populate_instance
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import dbg


def replace_quotes(string):
    # 文字列内に"data"と"timestamp"があるかどうかを確認
    if 'data' in string and 'timestamp' in string:
        data_idx = string.find('data')
        time_idx = string.find('timestamp')
        
        # "data"と"timestamp"の間の部分を取得
        target_str = string[time_idx:]
        
        # シングルクォーテーションに変換する位置を計算
        count = 0
        for idx, char in enumerate(target_str):
            if char == '"':
                count += 1
                if count in [1, 2]:
                    target_str = target_str[:idx] + "'" + target_str[idx+1:]
        
        # 変換後の文字列を結合して返す
        return string[:time_idx] + target_str
    
    # 条件を満たさない場合は元の文字列をそのまま返す
    return string


def create_bridge(
    factory: Union[str, "Bridge"],
    msg_type: str,
    topic_from: str,
    topic_to: str,
    frequency: Optional[float] = None,
    **kwargs
) -> "Bridge":
    """generate bridge instance using factory callable and arguments. if `factory` or `msg_type` is provided as string,
    this function will convert it to a corresponding object.
    """
    if isinstance(factory, str):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, str):
        msg_type = lookup_object(msg_type)
    # if not issubclass(msg_type, rospy.Message): # replace this with ROS2 once a solution for this esists
    #    raise TypeError(
    #        "msg_type should be rospy.Message instance or its string"
    #        "reprensentation")
    return factory(
        topic_from=topic_from,
        topic_to=topic_to,
        msg_type=msg_type,
        frequency=frequency,
        **kwargs
    )


class Bridge(object, metaclass=ABCMeta):
    """Bridge base class"""

    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr("serializer")
    _deserialize = inject.attr("deserializer")
    _extract_private_path = inject.attr("mqtt_private_path_extractor")


class RosToMqttBridge(Bridge):
    """Bridge from ROS topic to MQTT
    bridge ROS messages on `topic_from` to MQTT topic `topic_to`. expect `msg_type` ROS message type.
    """

    def __init__(
        self,
        topic_from: str,
        topic_to: str,
        msg_type,
        frequency: Optional[float] = None,
        **kwargs
    ):
        self.ros_node = kwargs["ros_node"]
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._last_published = self.ros_node.get_clock().now()
        self._interval = (
            Duration(seconds=0)
            if frequency is None
            else Duration(seconds=(1.0 / frequency))
        )
        self.ros_node.create_subscription(msg_type, topic_from, self._callback_ros, 1)

    def _callback_ros(self, msg):
        # self.ros_node.get_logger().info(
        #     "ROS received from {}".format(self._topic_from)
        # )
        now = self.ros_node.get_clock().now()
        if now - self._last_published >= self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg):
        """_summary_
        エッジ側は全てString()で投げるのでdataを取り出す機能を追加
        """
        payload_str = self._serialize(extract_values(msg))
        payload_dict = eval(payload_str)
        payload = payload_dict["data"].replace("'", '"')  # シングルをダブルに
        payload = replace_quotes(payload)
        self._mqtt_client.publish(topic=self._topic_to, payload=payload)


class MqttToRosBridge(Bridge):
    """Bridge from MQTT to ROS topic
    bridge MQTT messages on `topic_from` to ROS topic `topic_to`. MQTT messages will be converted to `msg_type`.
    """

    def __init__(
        self,
        topic_from: str,
        topic_to: str,
        msg_type,
        frequency: Optional[float] = None,
        queue_size: int = 10,
        **kwargs
    ):
        self.ros_node = kwargs["ros_node"]
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = self.ros_node.get_clock().now()
        self._interval = (
            None if frequency is None else Duration(seconds=(1.0 / frequency))
        )
        # Adding the correct topic to subscribe to
        self._mqtt_client.subscribe(self._topic_from)
        self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._publisher = self.ros_node.create_publisher(
            self._msg_type, self._topic_to, 10
        )  # , queue_size=self._queue_size)

    def _callback_mqtt(
        self, client: mqtt.Client, userdata: Dict, mqtt_msg: mqtt.MQTTMessage
    ):
        """callback from MQTT"""
        # self.ros_node.get_logger().info("MQTT received from {}".format(mqtt_msg.topic))
        now = self.ros_node.get_clock().now()

        if self._interval is None or now - self._last_published >= self._interval:
            try:

                ros_msg = self._create_ros_message(mqtt_msg)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                self.ros_node.get_logger().error(str(e))

    def _create_ros_message(self, mqtt_msg: mqtt.MQTTMessage):
        """create ROS message from MQTT payload"""
        # Hack to enable both, messagepack and json deserialization.
        if self._serialize.__name__ == "packb":
            msg_dict = self._deserialize(mqtt_msg.payload, raw=False)
        else:
            msg_dict = self._deserialize(mqtt_msg.payload)


        if "data" not in msg_dict:
            msg_dict_ = {}
            msg_dict_["data"] = str(msg_dict)
            return populate_instance(msg_dict_, self._msg_type())

        return populate_instance(msg_dict, self._msg_type())


__all__ = ['create_bridge', 'Bridge', 'RosToMqttBridge', 'MqttToRosBridge']