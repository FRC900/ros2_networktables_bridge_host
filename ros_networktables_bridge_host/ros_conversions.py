import json
from importlib import import_module

from rosidl_runtime_py import message_to_ordereddict
from rclpy_message_converter.message_converter import convert_ros_message_to_dictionary

def ros_msg_to_msg_dict(msg):
    msg_dict = message_to_ordereddict(msg)
    pkg = msg.__class__.__module__.split('.')[0]
    msg_dict['_type'] = f"{pkg}/{msg.__class__.__name__}"
    return msg_dict


def ros_msg_to_string_json(msg):
    msg_dict = ros_msg_to_msg_dict(msg)
    return json.dumps(msg_dict)


def string_json_to_msg_dict(msg_json: str):
    return json.loads(msg_json)


def remove_type_fields(msg_dict: dict):
    new_dict = {}
    for key in msg_dict.keys():
        if key == "_type":
            continue
        elif isinstance(msg_dict[key], dict):
            new_dict[key] = remove_type_fields(msg_dict[key])
        elif isinstance(msg_dict[key], list):
            new_dict[key] = []
            if len(msg_dict[key]) > 0:
                # ROS arrays are all the same type. Only check the first value if it has a _type field
                first_value = msg_dict[key][0]
                if isinstance(first_value, dict):
                    for value in msg_dict[key]:
                        assert isinstance(value, dict), value
                        new_dict[key].append(remove_type_fields(value))
                else:
                    new_dict[key] = msg_dict[key]
        else:
            new_dict[key] = msg_dict[key]
    return new_dict


def msg_dict_to_ros_type(msg_dict):
    if "_type" not in msg_dict:
        raise ValueError("JSON message must include a '_type' field.")

    msg_type = msg_dict["_type"]
    msg_dict = remove_type_fields(msg_dict)

    connection_header = msg_type.split('/')
    pkg = connection_header[0]
    msg_type_name = connection_header[-1]
    ros_pkg = pkg + '.msg'

    try:
        msg_class = getattr(import_module(ros_pkg), msg_type_name)
    except Exception:
        return None, None

    return msg_class, msg_dict


def msg_dict_to_ros_msg(msg_dict, msg_cls):
    assert msg_cls is not None
    return convert_ros_message_to_dictionary(msg_cls, msg_dict)

     
def string_json_to_ros_msg(string_json: str):
    if len(string_json) == 0:
        return None, None
    msg_dict = string_json_to_msg_dict(string_json)
    ros_msg_type, msg_dict = msg_dict_to_ros_type(msg_dict)
    if ros_msg_type is None or msg_dict is None:
        return None, None
    ros_msg = msg_dict_to_ros_msg(msg_dict, ros_msg_type)
    return ros_msg, ros_msg_type


def parse_nt_topic(nt_ros_topic: str) -> str:
    return nt_ros_topic.replace('\\', '/')


def convert_to_nt_topic(ros_topic: str) -> str:
    return ros_topic.replace('/', '\\')


def get_msg_class(cache, msg_type_name: str):
    if msg_type_name in cache:
        return cache[msg_type_name]
    connection_header = msg_type_name.split('/')
    pkg = connection_header[0]
    msg_type = connection_header[-1]
    ros_pkg = pkg + '.msg'
    msg_class = getattr(import_module(ros_pkg), msg_type)
    cache[msg_type_name] = msg_class
    return cache[msg_type_name]
