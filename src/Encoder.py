import pigpio
import RPi.GPIO as GPIO
import time
import math
import board
import busio
from math import atan2, sqrt
import multiprocessing


class EncoderCounter:
	channelA = 9
	channelB = 11
	const = (2 * math.pi * 2)

	def __init__(self):

		self.pi = pigpio.pi()
		self.count = 0
		self.x = 0
		self.y = 0
		self.dx = 0
		self.dy = 0
		self.error_x = 0
		self.error_y = 0


	def get_position(self, heading, counter):

		revolution = (counter/2060)  #168
		distance_cm = revolution * self.const
		change = distance_cm - self.prev_distance
		self.dx = math.cos(math.radians(heading)) * change
		self.dy = math.sin(math.radians(heading)) * change

		self.x += self.dx
		self.y += self.dy

		error_x = self.x - math.cos(math.radians(heading)) * distance_cm
		error_y = self.y - math.sin(math.radians(heading)) * distance_cm

		self.error_x = error_x
		self.error_y = error_y

		self.prev_distance = distance_cm

		return self.x, self.y

