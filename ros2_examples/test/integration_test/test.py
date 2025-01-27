import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
import rclpy
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from std_msgs.msg import Bool

def generate_test_description():

    ld = launch.LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_examples'),
                    'launch',
                    'publisher_example.py'
                    ])
                ]),
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_examples'),
                    'launch',
                    'subscriber_example.py'
                    ])
                ]),
            )
        )

    # starts the integration interactor
    ld.add_action(
            # Nodes under test
            launch_ros.actions.Node(
                package='ros2_examples',
                namespace='',
                executable='test_integration_test',
                name='integration_test',
            )
        )

    # starts the python test part down below
    ld.add_action(
        launch.actions.TimerAction(
            period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        )

    return ld

# Active tests
class PublisherHandlerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('integration_test_handler')

    def tearDown(self):
        self.node.destroy_node()

    # test if the logs contain certain messages
    def test_logs_spawning(self, proc_output):

        proc_output.assertWaitFor('receiving string', timeout=5.0, stream='stderr')

    def test_interactor(self, proc_output, timeout=20):

        """Check whether pose messages published"""

        test_result = []

        sub = self.node.create_subscription(
                Bool, '/test_result',
                lambda msg: test_result.append(msg), 100)
        try:

            end_time = time.time() + timeout

            while time.time() < end_time:

                if len(test_result) > 0:
                    break

                rclpy.spin_once(self.node, timeout_sec=1)

            # check if we have the result
            self.assertTrue(len(test_result) > 0)

            # check if the result is true
            self.assertTrue(test_result[0].data)

        finally:
            self.node.destroy_subscription(sub)

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class PublisherHandlerTestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
