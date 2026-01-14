import rclpy
from rclpy.node import Node
from typing import Callable
from ros_conversions import get_msg_class


class GenericMessageSubscriber:
    def __init__(self, node: Node, topic_name: str, callback: Callable, **kwargs):
        self._node = node
        self._topic = topic_name
        self._callback = callback
        self._qos = kwargs.get('qos_profile', 10)
        self._sub = self._node.create_subscription(msg_class, self._topic, self._callback, self._qos)

    def unregister(self):
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:
            pass
        self._sub = None
