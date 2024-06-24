import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        os.environ["DATA_TEXT_DIR"], "config", "pana_mqtt_config", "params.yaml"
    )
    config1 = os.path.join(
        os.environ["DATA_TEXT_DIR"], "config", "pana_mqtt_config", "tls_params.yaml"
    )


    node = Node(
        package="mqtt_bridge",
        name="pana_mqtt_bridge_node",
        executable="mqtt_bridge_node",
        parameters=[config, config1],
    )
    ld.add_action(node)
    return ld
