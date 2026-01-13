#!/usr/bin/env python3
import sys
import time
from typing import Any, Dict, Optional, Set
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from networktables import (
    NetworkTables,
    NetworkTable,
    NetworkTablesInstance,
    NetworkTableEntry,
)

from ros_conversions import (
    ros_msg_to_string_json,
    string_json_to_ros_msg,
    parse_nt_topic,
)
from generic_message_subscriber import GenericMessageSubscriber


NotifyFlags = NetworkTablesInstance.NotifyFlags


class ROSNetworkTablesBridge(Node):
    """
    This class is a bridge between ROS (Robot Operating System) and NetworkTables,
    allowing messages from ROS topics to be forwarded to NetworkTables and vice versa.
    The bridge can operate as either a NetworkTables server or a client,
    connecting to a remote server.
    """
    
    def __init__(self):
        super().__init__("ros_networktables_bridge")

        # Declare params
        self.declare_parameter('is_server', False)
        self.declare_parameter('address', '')
        self.declare_parameter('port', 1735)
        self.declare_parameter('update_interval', 0.02)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('nt_warmup_time', 2.0)
        self.declare_parameter('publish_warmup_time', 2.0)
        self.declare_parameter('publisher_non_empty_timeout', 0.25)
        self.declare_parameter('no_data_timeout', 2.0)
        self.declare_parameter('ros_to_nt_table_key', 'ros_to_nt')
        self.declare_parameter('nt_to_ros_table_key', 'nt_to_ros')
        self.declare_parameter('topics_request_key', '@topics')
        self.declare_parameter('time_sync_key', '@time')

        self.is_server = self.get_parameter('is_server').value
        self.address = str(self.get_parameter('address').value).strip()
        self.port = int(self.get_parameter('port').value)
        self.update_interval = float(self.get_parameter('update_interval').value)
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.nt_warmup_time = float(self.get_parameter('nt_warmup_time').value)
        self.publish_warmup_time = float(self.get_parameter('publish_warmup_time').value)
        self.publisher_non_empty_timeout = float(self.get_parameter('publisher_non_empty_timeout').value)
        self.no_data_timeout = float(self.get_parameter('no_data_timeout').value)
        self.ros_to_nt_key = str(self.get_parameter('ros_to_nt_table_key').value)
        self.nt_to_ros_key = str(self.get_parameter('nt_to_ros_table_key').value)
        self.topics_request_key = str(self.get_parameter('topics_request_key').value)
        self.time_sync_key = str(self.get_parameter('time_sync_key').value)

        self.open()

        # Initialize dictionaries and sets for publishers, subscribers, and keys
        self.nt_publishers: Dict[str, Any] = {}
        self.pub_listen_keys: Set[str] = set()
        self.pub_listen_handles: Dict[str, int] = {}
        self.pub_initial_values: Optional[Dict[str, Any]] = {}
        self.pub_lock = Lock()

        self.subscribers: Dict[str, GenericMessageSubscriber] = {}
        self.sub_listen_keys: Set[str] = set()

        # ROS namespace
        self.namespace: str = self.get_namespace()

        # start time of node (seconds)
        self.start_time = self.now()

        self.no_entries_timer = self.now()

        self.get_logger().info(f"Bridge namespace: {self.namespace}")
        self.get_logger().info("ros_networktables_bridge is ready.")

        self._timer = None # created in run

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def open(self):
        # Start NetworkTables server or client based on the is_server parameter
        if self.is_server:
            NetworkTables.startServer(listenAddress=self.address, port=self.port)
            self.get_logger().info(f"Starting server on port {self.port}")
            if len(self.address) > 0:
                self.get_logger().info(f"Address is {self.address}")
            assert NetworkTables.isServer()
        else:
            NetworkTables.startClient((self.address, self.port))
            self.get_logger().info(f"Connecting to {self.address}:{self.port}")
        NetworkTables.setUpdateRate(self.update_interval)

        # Get NetworkTables subtables
        self.ros_to_nt_subtable: NetworkTable = NetworkTables.getTable(self.ros_to_nt_key)
        self.nt_to_ros_subtable: NetworkTable = NetworkTables.getTable(self.nt_to_ros_key)
        self.ros_to_nt_topic_requests_table: NetworkTable = self.ros_to_nt_subtable.getSubTable(self.topics_request_key)
        self.nt_to_ros_topic_requests_table: NetworkTable = self.nt_to_ros_subtable.getSubTable(self.topics_request_key)
        self.time_sync_entry: NetworkTableEntry = self.ros_to_nt_subtable.getEntry(self.time_sync_key)

    def close(self):
        # Shutdown NetworkTables connection
        if self.is_server:
            NetworkTables.stopServer()
        else:
            NetworkTables.stopClient()

    def reopen(self):
        # Close and open NetworkTables connection
        self.close()
        time.sleep(0.25)
        self.open()
        time.sleep(self.nt_warmup_time)

    def update_time_sync(self):
        # Update time sync entry with current local time
        self.time_sync_entry.setDouble(self.now())

    def has_entries(self) -> bool:
        # Are there any entries or subtables in the ROS <-> NT tables
        num_keys = (
            len(self.ros_to_nt_subtable.getKeys())
            + len(self.nt_to_ros_subtable.getKeys())
            + len(self.ros_to_nt_subtable.getSubTables())
            + len(self.nt_to_ros_subtable.getSubTables())
        )
        return num_keys > 0

    def create_ros_to_nt_subscriber(self, nt_key: str):
        """
        Create a ROS subscriber that listens to a ROS topic. The subscriber's callback function forwards messages
        to a NetworkTables key.
        :param nt_key: the networktables key that triggered the subscription
        :param topic_name: Name of the ROS topic. Must be in the root namespace (starts with /)
        """
        assert len(nt_key) > 0
        topic_name = self.absolute_topic_from_nt_key(nt_key)
        assert len(topic_name) > 0

        # Check if subscriber has already been created
        if nt_key in self.subscribers:
            self.unregister_subscriber(nt_key)

        # create GenericMessageSubscriber with a callback to ros_to_nt_callback
        subscriber = GenericMessageSubscriber(self, topic_name, lambda msg: self.ros_to_nt_callback(msg, nt_key), queue_size=self.queue_size)

        # add to dictionary of subscribers
        self.subscribers[nt_key] = subscriber
        self.get_logger().info(f"Registering new subscriber: {topic_name} -> {nt_key}")

    def ros_to_nt_callback(self, msg, nt_key: str):
        """
        Callback function for forwarding ROS messages to NetworkTables.
        :param msg: ROS message.
        :param nt_key: the networktables key that triggered the subscription
        """
        try:
            # convert ROS message to JSON string
            raw_msg = ros_msg_to_string_json(msg)
            
            # send JSON string to corresponding NT entry
            self.ros_to_nt_subtable.getEntry(nt_key).setString(raw_msg)
        except Exception as e:
            self.get_logger().error(f"Exception in ros_to_nt_callback: {e}")

    def create_nt_to_ros_publisher(self, nt_key: str, ros_msg_type) -> None:
        """
        Create a ROS publisher that listens to a NetworkTables key and forwards messages to a ROS topic.
        :param topic_name: Name of the ROS topic.
        :param ros_msg_type: ROS message type.
        """
        topic_name = self.absolute_topic_from_nt_key(nt_key)
        assert len(topic_name) > 0

        # Check if publisher has already been created
        if nt_key in self.nt_publishers:
            self.unregister_publisher(nt_key)

        # create ROS publisher using message type received from NT
        qos = QoSProfile(depth=self.queue_size)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # add to dictionary of publishers
        pub = self.create_publisher(ros_msg_type, topic_name, qos)
        self.nt_publishers[nt_key] = pub
        self.get_logger().info(f"Registering new publisher: {nt_key} -> {topic_name}")

    def get_key_base_name(self, key: str) -> str:
        # remove prefix from the key
        table_divider = key.rfind("/")
        return key[table_divider + 1 :]

    def absolute_topic_from_nt_key(self, key: str) -> str:
        topic_key = self.get_key_base_name(key)
        
        # convert to ROS topic
        topic_name = parse_nt_topic(topic_key)
        
        # convert to absolute topic
        topic_name = self.get_absolute_topic(topic_name)
        return topic_name

    def nt_to_ros_callback(self, entry: NetworkTableEntry, key: str, value: str, isNew: int):
        """
        Callback function for forwarding NetworkTables messages to ROS.
        :param entry: NetworkTableEntry object.
        :param key: NetworkTables key.
        :param value: NetworkTables value.
        :param isNew: Flag indicating whether the value is new.
        """
        try:
            with self.pub_lock:
                self.publish_ros_message_from_nt_string(key, value)
        except Exception as e:
            self.get_logger().error(f"Exception in nt_to_ros_callback: {e}")

    def publish_ros_message_from_nt_string(self, key: str, value: str) -> None:
        key = self.get_key_base_name(key)
        assert len(key) > 0

        # convert JSON string to ROS message and type
        ros_msg, ros_msg_type = string_json_to_ros_msg(value)
        if ros_msg is None or ros_msg_type is None:
            self.get_logger().warn(f"Failed to parse message from {key}: {value}")
            return
        
        # create the publisher if that hasn't been done yet
        if key not in self.nt_publishers:
            self.create_nt_to_ros_publisher(key, ros_msg_type)
            
        # get the corresponding publisher to the topic name
        pub = self.nt_publishers[key]

        try:
            if self.pub_initial_values is None:
                # If initial values is None, the warmup time has exceeded.
                # Publish all new values
                pub.publish(ros_msg)
            else:
                if key in self.pub_initial_values:
                    # If the we're still in warmup time, if an initial value has been registered
                    # already, that means the value was updated during warmup. Publish this
                    # new value
                    pub.publish(ros_msg)
                else:
                    # Otherwise, it's possible this value is stale, so don't publish it.
                    self.pub_initial_values[key] = ros_msg
                    self.get_logger().debug(f"Initial value {key}: {ros_msg}")

            # remove initial values if start up time is exceeded.
            # This is done to save memory in case the initial value is large
            if self.pub_initial_values is not None and self.now() - self.start_time > self.publish_warmup_time:
                self.get_logger().debug("Publish warmup complete")
                self.pub_initial_values = None
        except Exception as e:
            self.get_logger().error(f"Exception publishing message: {e}")

    def unregister_publisher(self, nt_key: str) -> None:
        """
        Unregister a publisher by NetworkTables entry key.

        :param nt_key: NetworkTables entry key that points to a publisher
        """
        self.get_logger().info(f"Unregistering publish topic {nt_key}")
        if nt_key not in self.nt_publishers:
            self.get_logger().warn(f"Can't remove publisher {nt_key} as it's already removed.")
            return
        pub = self.nt_publishers[nt_key]
        try:
            self.destroy_publisher(pub)
        except Exception:
            pass
        del self.nt_publishers[nt_key]

    def unregister_subscriber(self, nt_key: str) -> None:
        """
        Unregister a subscriber by NetworkTables entry key.

        :param nt_key: NetworkTables entry key that points to a subscriber
        """
        self.get_logger().info(f"Unregistering subscribe topic {nt_key}")
        if nt_key not in self.subscribers:
            self.get_logger().warn(f"Can't remove subscriber {nt_key} as it's already removed.")
            return
        self.subscribers[nt_key].unregister()
        del self.subscribers[nt_key]

    def get_new_keys(self, local_keys: Set[str], remote_keys: Set[str]) -> Set[str]:
        """
        Comparing local NetworkTables keys and remote NetworkTables keys, get the keys that are new

        :return: Set of new keys
        """
        return remote_keys.difference(local_keys)

    def get_removed_keys(self, local_keys: Set[str], remote_keys: Set[str]) -> Set[str]:
        """
        Comparing local NetworkTables keys and remote NetworkTables keys, get the keys that were removed

        :return: Set of removed keys
        """
        return local_keys.difference(remote_keys)

    def get_topic_requests(self, topic_requests_table: NetworkTable) -> Set[str]:
        """
        Get a set of topics from the supplied topic requests table.

        :return: Set of requested topic keys
        """
        requests = set()
        for entry_name in topic_requests_table.getKeys():
            entry = topic_requests_table.getEntry(entry_name)
            if entry.getBoolean(False):
                requests.add(entry_name)
        return requests

    def get_absolute_topic(self, topic_name: str) -> str:
        """
        Get topic in the root namespace.

        :param topic_name: absolute or relative topic
        :return: absolute topic
        """
        if topic_name.startswith("/"):
            return topic_name
        return ns + topic_name

    def wait_for_non_empty(self, entry: NetworkTableEntry) -> str:
        start_time = time.time()
        value = ""
        while len(value) == 0:
            if (not rclpy.ok()) or (time.time() - start_time > self.publisher_non_empty_timeout):
                return ""
            value = entry.getString("")
        return value

    def check_new_pub_keys(self, new_pub_keys: Set[str]) -> None:
        for new_pub_key in new_pub_keys:
            # if there's already an entry listener for this new topic, skip it
            if new_pub_key in self.pub_listen_keys:
                continue

            # Add to set of initialized keys
            self.pub_listen_keys.add(new_pub_key)

            # forcefully call the callback since this is the first update
            new_pub_entry = self.nt_to_ros_subtable.getEntry(new_pub_key)
            self.nt_to_ros_callback(new_pub_entry, new_pub_key, self.wait_for_non_empty(new_pub_entry), True)

            # create an entry listener for the new topic.
            handle = new_pub_entry.addListener(self.nt_to_ros_callback, NotifyFlags.UPDATE)
            self.pub_listen_handles[new_pub_key] = handle

    def check_removed_pub_keys(self, removed_pub_keys: Set[str]) -> None:
        for removed_pub_key in removed_pub_keys:
            # if the topic was already removed, skip it
            if removed_pub_key not in self.pub_listen_keys:
                continue

            # Remove from initialized keys
            self.pub_listen_keys.remove(removed_pub_key)

            removed_pub_entry = self.nt_to_ros_subtable.getEntry(removed_pub_key)
            removed_pub_entry.removeListener(self.pub_listen_handles[removed_pub_key])
            del self.pub_listen_handles[removed_pub_key]

            with self.pub_lock:
                self.unregister_publisher(removed_pub_key)

    def check_new_sub_keys(self, new_sub_keys: Set[str]) -> None:
        for new_sub_key in new_sub_keys:
            # if there's already an subscriber for this new topic, skip it
            if new_sub_key in self.sub_listen_keys:
                continue

            # Add to set of initialized keys
            self.sub_listen_keys.add(new_sub_key)

            # create a subscriber for the new topic
            self.create_ros_to_nt_subscriber(new_sub_key)

    def check_removed_sub_keys(self, removed_sub_keys: Set[str]) -> None:
        for removed_sub_key in removed_sub_keys:
            # if the topic was already removed, skip it
            if removed_sub_key not in self.sub_listen_keys:
                continue

            # Remove from initialized keys
            self.sub_listen_keys.remove(removed_sub_key)

            self.unregister_subscriber(removed_sub_key)

    def _on_timer(self):
        """
        Main loop of the ROSNetworkTablesBridge. Continuously checks for new NetworkTables keys to create
        publishers and subscribers.
        """
        # push local time to NT
        self.update_time_sync()

        if self.has_entries():
            self.no_entries_timer = self.now()
        else:
            if self.now() - self.no_entries_timer > self.no_data_timeout:
                self.get_logger().error(f"No data received for {self.no_data_timeout} seconds. Exiting.")
                rclpy.shutdown()
                sys.exit(1)

        # get new publisher topics requested by the NT client
        pub_topic_requests = self.get_topic_requests(self.nt_to_ros_topic_requests_table)
        new_pub_keys = self.get_new_keys(self.pub_listen_keys, pub_topic_requests)
        self.check_new_pub_keys(new_pub_keys)

        # get removed publisher topics requested by the NT client
        removed_pub_keys = self.get_removed_keys(self.pub_listen_keys, pub_topic_requests)
        self.check_removed_pub_keys(removed_pub_keys)

        # get new subscriber topics requested by the NT client
        sub_topic_requests = self.get_topic_requests(self.ros_to_nt_topic_requests_table)
        new_sub_keys = self.get_new_keys(self.sub_listen_keys, sub_topic_requests)
        self.check_new_sub_keys(new_sub_keys)

        # get removed subscriber topics requested by the NT client
        removed_sub_keys = self.get_removed_keys(self.sub_listen_keys, sub_topic_requests)
        self.check_removed_sub_keys(removed_sub_keys)

    def run(self):
        # wait for NT entries to populate and then initialize timers (rate limit)
        time.sleep(self.nt_warmup_time)
        self.start_time = self.now()

        # initialize no entries timer
        self.no_entries_timer = self.now()

        self._timer = self.create_timer(self.update_interval, self._on_timer)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    bridge = ROSNetworkTablesBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
