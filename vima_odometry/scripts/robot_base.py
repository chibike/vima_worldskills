#!/usr/bin/env python

'''
This program was written by Chibuike Okpaluba and Marlon Gwira.

For more information please contact <##>
Thank you.

Copyright 2017 Middlesex
'''

import time
import math
import md25
import rospy
import generic_functions
import wiringpi2 as wpi

from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class RobotBase(object):
	def __init__(self):
		# Get parameters
		r = rospy.get_param("~rate", 10)
		md25_address   = rospy.get_param("~md25_address", 0x58)
		self.wheelbase_diameter = rospy.get_param("~base_width", 0.265)
		self.gpio_pin  = rospy.get_param("~gpio_pin", 7)

		# Setup Start button
		wpi.wiringPiSetup()
		wpi.pinMode(self.gpio_pin, wpi.GPIO.INPUT)

		# Setup md25
		self.wheels_controller = None
		while self.wheels_controller == None:
			try:
				self.wheels_controller = md25.MD25(md25_address)
			except ValueError as e:
				print "Has ValueError", e
				self.wheels_controller = None
			except IOError as e:
				print "Has ValueError", e
				self.wheels_controller = None

		rospy.init_node('robot_base', anonymous=True)
		self.rate = rospy.Rate(r)

		self.left_encoder_data_publisher  = rospy.Publisher('lwheel', Int32, queue_size=10)
		self.right_encoder_data_publisher = rospy.Publisher('rwheel', Int32, queue_size=10)

		self.cmd_vel_publisher = rospy.Publisher('twist', Twist, queue_size=10)

		self.battery_voltage_publisher     = rospy.Publisher('battery_voltage', Float32, queue_size=10)
		self.left_motor_current_publisher  = rospy.Publisher('lmotor_current', Float32, queue_size=10)
		self.right_motor_current_publisher = rospy.Publisher('rmotor_current', Float32, queue_size=10)
		
		self.start_btn_state_publisher = rospy.Publisher('start_btn', Bool, queue_size=10)

		rospy.Subscriber('lmotor_cmd', Float32, self.left_motor_control_callback)
		rospy.Subscriber('rmotor_cmd', Float32, self.right_motor_control_callback)
		
		rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleop_callback)
		rospy.Subscriber('cmd_vel', Twist, self.teleop_callback)

	def publish_encoder_counts(self):
		data = {}
		try:
			data = self.wheels_controller.get_encoder_counts()
		except IOError as e:
			print "Has IOError",e
			return

		self.left_encoder_data_publisher.publish(data['left_count'])
		self.right_encoder_data_publisher.publish(data['right_count'])

	def publish_start_btn_state(self):
		state = wpi.digitalRead(7)
		self.start_btn_state_publisher.publish(state)

	def publish_power_info(self):
		data = {}
		try:
			data = self.wheels_controller.get_motor_currents()
		except IOError as e:
			print "Has IOError",e
			return
		
		self.left_motor_current_publisher.publish(data['left_current'])
		self.right_motor_current_publisher.publish(data['right_current'])
		self.battery_voltage_publisher.publish(self.wheels_controller.get_input_voltage())

	def left_motor_control_callback(self, data):
		wheel_power = data.data
		try:
			self.wheels_controller.set_wheel_speed_left(wheel_power)
		except IOError as e:
			print "Has IOError",e
			return
		except ValueError as e:
			print "Has ValueError", e
			return

	def right_motor_control_callback(self, data):
		wheel_power = data.data
		try:
			self.wheels_controller.set_wheel_speed_right(wheel_power)
		except IOError as e:
			print "Has IOError",e
			return
		except ValueError as e:
			print "Has ValueError", e
			return

	def teleop_callback(self, data):
		self.cmd_vel_publisher.publish(data)

	def run(self):
		while not rospy.is_shutdown():
			self.publish_encoder_counts()
			self.publish_start_btn_state()
			self.publish_power_info()
			self.rate.sleep()

if __name__ == '__main__':
	# TODO: Add try-except
	my_robot_base = RobotBase()
	my_robot_base.run()