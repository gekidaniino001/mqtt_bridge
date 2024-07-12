import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from pathlib import Path


# 置き換えを行う関数
def replace_config(value,mqtt_detail):
    if isinstance(value, dict):
        return {k: replace_config(v, mqtt_detail) for k, v in value.items()}
    elif isinstance(value, list):
        return [replace_config(i, mqtt_detail) for i in value]
    elif isinstance(value, str):
        return value.replace('FACI/AREA/MOB', mqtt_detail)
    else:
        return value

def generate_launch_description():
    config_path = os.path.join(
        os.environ["DATA_TEXT_DIR"], "config", "mqtt_config", "params_base.yaml"
    )


    yaml_file_path = os.environ["DATA_TEXT_DIR"]+"/config/map_car.yaml"

    try:
        with open(yaml_file_path, 'r') as yaml_file:
            map_car = yaml.safe_load(yaml_file)
    except:
        print("-----ERROR: MAP CAR YAMLの読み込みが出来ません")

    map_name = Path(map_car["map_path"]).name
    lanelet2_name = Path(Path(map_car["lanelet2_name"]).name).stem
    car = map_car["car"]
    print(map_name,lanelet2_name,car)

    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        config["mqtt_bridge_node"]["ros__parameters"]["mqtt"]["client"]["client_id"] = car
        updated_config = replace_config(config, "/".join([map_name,lanelet2_name,car]))
    except:
        print("置き換え失敗")
    print(updated_config)

    tls = os.path.join(
        os.environ["DATA_TEXT_DIR"], "config", "mqtt_config", "tls_params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="mqtt_bridge",
                name="mqtt_bridge_node",
                executable="mqtt_bridge_node",
                parameters=[updated_config["mqtt_bridge_node"]["ros__parameters"], tls],
                output="screen",
            ),
            Node(
                package="iino_common",
                executable="vehicle_current",
                # output="screen",
            ),
            Node(
                package="iino_common",
                executable="rcio",
                # output="screen",
            ),
            Node(
                package="iino_common",
                executable="pub_status_for_mims",
                # output="screen",
            ),
            Node(
                package="iino_common",
                executable="alert_for_mims",
                # output="screen",
            ),
            Node(
                package="iino_common",
                executable="hb_mims_chk",
            ),
            Node(
                package="iino_common",
                executable="mims_joy",
            ),
        ]
    )