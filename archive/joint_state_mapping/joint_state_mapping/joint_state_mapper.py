#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from interface_protocol.msg import JointState as HardwareJointState
from sensor_msgs.msg import JointState as RosJointState

class JointStateMapper(Node):
    def __init__(self):
        super().__init__('joint_state_mapper')

        # Exact matching joint names from your URDF:
        self.joint_names = [
            "J00_HIP_PITCH_L", "J01_HIP_ROLL_L", "J02_HIP_YAW_L", "J03_KNEE_PITCH_L","J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L", 
            "J06_HIP_PITCH_R", "J07_HIP_ROLL_R", "J08_HIP_YAW_R", "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R",
            "J12_WAIST_YAW",  "J13_SHOULDER_PITCH_L", "J14_SHOULDER_ROLL_L", "J15_SHOULDER_YAW_L", "J16_ELBOW_PITCH_L", "J17_ELBOW_YAW_L",
            "J18_SHOULDER_PITCH_R", "J19_SHOULDER_ROLL_R", "J20_SHOULDER_YAW_R", "J21_ELBOW_PITCH_R", "J22_ELBOW_YAW_R", "J23_HEAD_YAW",
        ] 

        # Left out joints were: "J_FIXED_FOOT_L", "J_FIXED_FOOT_R", "J_FIXED_ELBOW_END_L", "J_FIXED_ELBOW_END_R",

        # ### SUBSCRIPTIONS

        self.joint_state_sub = self.create_subscription(
            HardwareJointState,
            '/hardware/joint_state',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        ### PUBLICATIONS
        self.joint_state_pub = self.create_publisher(
            RosJointState,
            '/joint_states',
            10
        )

        self.get_logger().info("Joint state mapper initialized. From /hardware/joint_states to /joint_states.")


    def joint_state_callback(self, source):

        if len(source.position) != len(self.joint_names):
            self.get_logger().error(
                f'Expected {len(self.joint_names)} positions, '
                f'received {len(source.position)}'
            )
            return

        output = RosJointState()
        output.header.stamp = source.header.stamp
        output.name = self.joint_names
        output.position = list(source.position)

        if len(source.velocity) == len(self.joint_names):
            output.velocity = list(source.velocity)

        if len(source.torque) == len(self.joint_names):
            output.effort = list(source.torque)

        self.joint_state_pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()