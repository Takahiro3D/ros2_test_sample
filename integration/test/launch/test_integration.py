import pytest
import launch_pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from pathlib import Path

import rclpy
import rclpy.node
from std_msgs.msg import Int32
from threading import Thread


@pytest.fixture
def integration_launch():
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("integration"))
                / "launch"
                / "integration.launch.yaml"
            )
        )
    )


@pytest.fixture
def testdata():
    path_to_test = Path(__file__).parent.parent

    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(path_to_test / "data" / "testdata"),
            "--delay",
            "1",
            "--wait-for-all-acked",
            "1000"
        ],
        shell=True
    )


@pytest.fixture
def receiver():
    rclpy.init()
    node = ReceiverNode()
    node.start()
    yield node
    node.destroy_node()
    rclpy.shutdown()


class ReceiverNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("test_receiver")
        self._msgs = []
        self._sub = self.create_subscription(
            Int32, "dst", lambda msg: self._msgs.append(msg), 10
        )

    def start(self):
        self._ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self._ros_spin_thread.start()


@launch_pytest.fixture
def generate_test_description(testdata, integration_launch):
    return LaunchDescription(
        [
            testdata,
            integration_launch
        ]
    )


@pytest.mark.launch(fixture=generate_test_description)
async def test_shoud_be_90(testdata, receiver):
    await testdata.get_asyncio_future()
    yield
    assert receiver._msgs[-1] == Int32(data=90)
