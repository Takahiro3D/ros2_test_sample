import launch
import launch_ros.actions

import pytest
import launch_pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import time
from threading import Thread

import domain_coordinator
import contextlib
import os

stack = contextlib.ExitStack()
if "ROS_DOMAIN_ID" not in os.environ and "DISABLE_ROS_ISOLATION" not in os.environ:
    domain_id = stack.enter_context(domain_coordinator.domain_id())
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)


@launch_pytest.fixture
def generate_test_desription():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="py_accum",
                executable="accum"
            )
        ]
    )


@pytest.mark.launch(fixture=generate_test_desription)
def test_accumulation(make_test_node):
    node = make_test_node
    for i in range(10):
        node.publish(Int32(data=i))
        time.sleep(0.01)

    end_time = time.time() + 5.0
    while time.time() < end_time:
        if len(node._msgs) == 10:
            break
        time.sleep(0.1)

    assert len(node._msgs) == 10
    assert node._msgs[-1] == Int32(data=45)


@pytest.fixture
def make_test_node():
    rclpy.init()
    node = DummyTestNode()
    node.start()
    yield node
    node.destroy_node
    rclpy.shutdown()


class DummyTestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self._msgs = []
        self._pub = self.create_publisher(Int32, "src", 10)
        self._sub = self.create_subscription(
            Int32, "dst", lambda msg: self._msgs.append(msg), 10
        )

    def _wait_for_connect(self, timeout_s=5.0):
        end_time = time.time() + timeout_s
        while time.time() < end_time:
            cnt = self._pub.get_subscription_count()
            if cnt > 0:
                return True
            time.sleep(0.1)
        return False

    def start(self):
        self._ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self._ros_spin_thread.start()
        self._wait_for_connect()

    def publish(self, data):
        self._pub.publish(data)
