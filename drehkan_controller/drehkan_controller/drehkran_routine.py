#!/usr/bin/env python3
import numpy as np
import time
import rclpy
from rclpy.time import Time
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

class DrehkranController(Node):
    def __init__(self):
        super().__init__('drehkran_controller')

        self._jog_topic = "drehkran/joint_jog"
        self._joint_state_topic = "drehkran/joint_state"
        self._joint_jog_publisher = None

        # JointState in order: [Achse1, Achse2, Achse3, Achse4, Achse5, Achse6, Achse7]
        self._joint_states = [0.0]*7
        self.max_velocities = [66.65, 5.0, 2.0, 12.0, 12.0, 0.0, 0.0]   #EFFECTIVE MAX VELOCITIES

        self._joint_jog_msg = JointJog()
        self._joint_jog_msg.header.frame_id = "JointJog"
        self._joint_jog_msg.joint_names = [
            "Achse1", # translation rail
            "Achse2", # rotation neagitve = clockwise
            "Achse3", # shoulder joint pitch
            "Achse4", # elbow joint pitch
            "Achse5", # spreader pitch
            "Achse6", # spreader yaw
            "Achse7", # spreader pulley
        ]

        self.i = 0
        self.time_start_actions = [0.0]*10

        # Define custom values
        # self.custom_values = [
        #     [0.0, 0.0, -1.85, -1.5, 3.85, 0.0, 0.0] # Achsen 3, 4, 5 – Heben
        #     [0.0, 5, 0.0, 0.0, 0.0, 0.0, 0.0] # Achse 2 – 180°
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Pause
        #     [0.0, 0.0, 0.8*MAX, 0.8*MAX, 0.8*MAX, 0.0, 0.0], # Achse 3, 4, 5 – Senken
        #     [0.0, 0.0, -0.8*MAX, -0.8*MAX, -0.8*MAX, 0.0, 0.0] # Achsen 3, 4, 5 – Heben
        #     [0.9*MAX, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Achse 1 – Vor
        #     [0.0, -0.8*MAX, 0.0, 0.0, 0.0, 0.0, 0.0] # Achse 2 – -180°
        #     [0.0, 0.0, 0.8*MAX, 0.8*MAX, 0.8*MAX, 0.0, 0.0] # Achse 3, 4, 5 – Senken
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Ende
        # ]

        self.node = rclpy.create_node("time_example")
        

        self.custom_values = [
            [0.0, 0.0, -1.85, -1.5, 3.85, 0.0, 0.0],
            [0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, +1.85, +1.5, -3.85, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.85, -1.5, 3.85, 0.0, 0.0],
            [0.0, +10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-65.0, .0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, +10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, +1.85, +1.5, -3.85, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        print(self.custom_values)

        self.durations_sec = [20.0, 18.4, 8.0, 3.0, 8.0, 9.2, 20.0, 9.2, 20.0, 5.0]
        self.moving = False
        # Set timer for sending joint jog at 50hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        # self.moving_timer = self.create_timer(0.05, self.moving_timer_callback)

        self._register_publishers()

        self.time_long = self.node.get_clock().now()
        print(self.time_long.nanoseconds/1000000000)
        self.time_start_actions[self.i] =  self.time_long.nanoseconds/1000000000

    def timer_callback(self):
        """Sending velocity of the axes at 50Hz"""
        self._send_joint_jog([float(val) for val in self.custom_values[self.i]], self.durations_sec[self.i])
        self.time_long = self.node.get_clock().now()
        time_difference = self.time_long.nanoseconds/1000000000 - self.time_start_actions[self.i]
        print(time_difference)

        if time_difference > self.durations_sec[self.i]:
            self.i = self.i +1
            self.time_start_actions[self.i] =  self.time_long.nanoseconds/1000000000
            if self.i == 10:
                print('finished the routine')
                self._unregister_publishers
            
    def _send_joint_jog(self, velocities, duration):
        if self._joint_jog_publisher is None:
            return
        self._joint_jog_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_jog_msg.velocities = velocities
        self._joint_jog_msg.duration = duration
        self._joint_jog_publisher.publish(self._joint_jog_msg)

    def _register_publishers(self):
        self._unregister_publishers()
        self._joint_jog_publisher = self.create_publisher(
            JointJog, self._jog_topic, qos_profile=QoSProfile(depth=1)
        )

    def _unregister_publishers(self):
        if self._joint_jog_publisher is not None:
            self.destroy_publisher(self._joint_jog_publisher)
            self._joint_jog_publisher = None

def main(args=None):
    rclpy.init(args=args)

    drehkran_controller = DrehkranController()
    rclpy.spin(drehkran_controller)

    drehkran_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
