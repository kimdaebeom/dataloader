"""Small ROS 1/ROS 2 compatibility layer used by the dataset player.

The public conversion and reader APIs do not import this module. It is loaded
only by :mod:`dataloader.player`, after a ROS environment has been sourced.
"""

import os
import time


def _format_message(message, args):
    return message % args if args else message


class _Ros2Publisher:
    def __init__(self, publisher):
        self._publisher = publisher
        self.name = publisher.topic_name

    def publish(self, message):
        self._publisher.publish(message)


class _Ros2Facade:
    def __init__(self):
        self._node = None
        self._last_warnings = {}

    def init_node(self, name):
        import rclpy

        rclpy.init()
        self._node = rclpy.create_node(
            name,
            automatically_declare_parameters_from_overrides=True,
        )

    def _require_node(self):
        if self._node is None:
            raise RuntimeError("ROS 2 node has not been initialized")
        return self._node

    def get_param(self, name, default=None):
        node = self._require_node()
        name = name.lstrip("~/").replace("/", ".")
        if node.has_parameter(name):
            return node.get_parameter(name).value

        prefix = name + "."
        result = {}
        for parameter_name in node.list_parameters([name], depth=100).names:
            if not parameter_name.startswith(prefix):
                continue
            target = result
            parts = parameter_name[len(prefix):].split(".")
            for part in parts[:-1]:
                target = target.setdefault(part, {})
            target[parts[-1]] = node.get_parameter(parameter_name).value
        return result or default

    def Publisher(self, topic, message_type, queue_size=10):
        publisher = self._require_node().create_publisher(
            message_type,
            topic,
            queue_size,
        )
        return _Ros2Publisher(publisher)

    def is_shutdown(self):
        import rclpy

        return not rclpy.ok()

    def signal_shutdown(self, reason=""):
        import rclpy

        if reason:
            self.loginfo("shutting down: %s", reason)
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def sleep(duration):
        time.sleep(duration)

    def loginfo(self, message, *args):
        self._require_node().get_logger().info(_format_message(message, args))

    def logwarn(self, message, *args):
        self._require_node().get_logger().warning(_format_message(message, args))

    def logwarn_throttle(self, period, message, *args):
        rendered = _format_message(message, args)
        now = time.monotonic()
        if now - self._last_warnings.get(message, float("-inf")) >= period:
            self._last_warnings[message] = now
            self.logwarn(rendered)


def _load_ros():
    ros_version = os.environ.get("ROS_VERSION")
    if ros_version == "1":
        import rospy

        return rospy
    if ros_version == "2":
        return _Ros2Facade()
    raise RuntimeError(
        "ROS_VERSION is not set. Source a ROS 1 or ROS 2 environment before "
        "running the dataset player."
    )


ros = _load_ros()


def stamp_from_ns(timestamp_ns):
    """Create the current ROS version's message timestamp without float loss."""

    timestamp_ns = int(timestamp_ns)
    if os.environ.get("ROS_VERSION") == "1":
        return ros.Time(
            secs=timestamp_ns // 1_000_000_000,
            nsecs=timestamp_ns % 1_000_000_000,
        )

    from rclpy.time import Time

    return Time(nanoseconds=timestamp_ns).to_msg()
