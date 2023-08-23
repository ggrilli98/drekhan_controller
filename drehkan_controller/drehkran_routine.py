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
        
        self.custom_values = [
            [0.0, 0.0, -1.85, -1.5, 3.85, 0.0, 0.0], #pickup water
            [0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0], #rotate 180 clockwise
            [0.0, 0.0, +1.85, +1.5, -3.85, 0.0, 0.0], #lowerdown
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], #standstill
            [0.0, 0.0, -1.85, -1.5, 3.85, 0.0, 0.0], #getup
            [0.0, +10.0, 0.0, 0.0, 0.0, 0.0, 0.0], #rotation 90 counterclockwise
            [-66.5, .0, 0.0, 0.0, 0.0, 0.0, 0.0], #move on the rail
            [0.0, +10.0, 0.0, 0.0, 0.0, 0.0, 0.0], #rotation 90 counterclockwise
            [0.0, 0.0, +1.85, +1.5, -3.85, 0.0, 0.0], #lowerdown water
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], #standstill and death
        ]

        self.durations_sec = [20.0, 18.4, 8.0, 3.0, 8.0, 9.2, 20.0, 9.2, 20.0, 5.0]
        
        self._register_publishers()
        self._register_subscribers() 

        self.i = 0
        self.node = rclpy.create_node("time_example")
        self.time_start_actions = [0.0]*10

        self.time_long = self.node.get_clock().now()
        print(self.time_long.nanoseconds/1000000000)
        self.time_start_actions[self.i] =  self.time_long.nanoseconds/1000000000
        
        self.timer = self.create_timer(0.05, self.timer_callback)

        #AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

        # POSITIONAL CONTROL
        self.encoder_positions = [0.0]*7
        self.desired_values_encoder = [0.0, 0.0, 82.0, 180.0, 165.0, 195.0, 44.0] 
        self.actual_error = [0.0]*7
        self.initial_error = [0.0]*7
        self.moving = [False]*7
        self.axis_velocities = [0.0]*7

    def joint_state_subscription_callback(self, msg):
        for i in range(enumerate(msg.name)):
            
            if i == 99: #change the number 99 according to the relative position in the joint_state msg of the fake axis
                counter = 1 #skip it maybe if the joint value is doubles in the joint state? don't remember how it's written and which joint exaclty
            self.encoder_positions[i] = msg.position[i + counter] #the counter makes it so that we increase by one the reading number, thus skipping the fake axis 
            self.actual_error[i] = self.encoder_positions - self.desired_values_encoder[i]

            if self.moving[i] == False:#memorizing the initial error for each axis
                self.initial_error[i] = self.actual_error[i]
                self.moving[i] = True

            if self.moving[i] == True: #if the axis has to move gettin the velocity token and then the final velocity
                velocity_token = self.get_velocity_token_from_encoder_error(self.actual_error[i], i)
                self.axis_velocities[i] = velocity_token[i]*self.max_velocities[i]

        self._send_joint_jog(self.axis_velocities, 5) #arbitrary 5 second duration

            
    def get_velocity_token_from_encoder_error(self, actual_encoder_error, index):
        # DEFINING IF THE MOTOR HAS TO GO BACKWARDS OR FORWARDS
        if actual_encoder_error < 0.0:
            velocity_sign = -1.0
        else:
            velocity_sign = 1.0
        velocity_token =  velocity_sign   
        #FIRST SLOW DOWN 
        if abs(actual_encoder_error) < 0.3* self.initial_error[index]:
            velocity_token = 0.8 * velocity_sign  
        #SECOND SLOW DOWN
        if abs(actual_encoder_error) < 0.1*self.initial_error[index]:
            velocity_token = 0.5 * velocity_sign   
        #STOPPING WHEN THE ERROR IS NOT RELEVANT
        if abs(actual_encoder_error) < 0.1:
            velocity_token = 0.0
            self.moving[index] = False #the axis has reached it's goal, but it's not said that the others did
        return velocity_token


    # AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            
                                
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
            JointJog, "drehkran/joint_jog", qos_profile=QoSProfile(depth=1)
        )

    def _unregister_publishers(self):
        if self._joint_jog_publisher is not None:
            self.destroy_publisher(self._joint_jog_publisher)
            self._joint_jog_publisher = None

    def _register_subscribers(self):

        self._state_subscriber = self.create_subscription(
            JointState,
            "drehkran/joint_state",
            self.joint_state_subscription_callback,
            qos_profile=1,
        )

def main(args=None):
    rclpy.init(args=args)

    drehkran_controller = DrehkranController()
    rclpy.spin(drehkran_controller)

    drehkran_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
