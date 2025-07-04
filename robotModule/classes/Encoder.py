import pigpio
import RPi.GPIO as GPIO
import time
import math
import board
import busio
from math import atan2, sqrt
import matplotlib.pyplot as plt
import multiprocessing


class EncoderCounter:
	channelA = 9
	channelB = 11
	const = (2 * math.pi * 2)

	def __init__(self):

		self.pi = pigpio.pi()

		self.pi.set_mode(self.channelA, pigpio.INPUT)
		self.pi.set_pull_up_down(self.channelA, pigpio.PUD_UP)  # channel A
		self.pi.set_mode(self.channelB, pigpio.INPUT)
		self.pi.set_pull_up_down(self.channelB, pigpio.PUD_UP)  # channel B

		self.prev_state_A = self.pi.read(self.channelA)
		self.prev_state_B = self.pi.read(self.channelB)
		self.count = 0
		self.x = 0
		self.y = 0
		self.dx = 0
		self.dy = 0
		self.error_x = 0
		self.error_y = 0
		self.x_history = [self.x]
		self.y_history = [self.y]
		self.prev_distance = 0

	def start_counter(self):

		'''self.cb_A = self.pi.callback(self.channelA, pigpio.EITHER_EDGE, self.edge_detected)
		self.cb_B = self.pi.callback(self.channelB, pigpio.EITHER_EDGE, self.edge_detected)'''
		self.prev_state_A = self.pi.read(self.channelA)
		self.prev_state_B = self.pi.read(self.channelB)

	def run_encoder(self):
		stateA = self.pi.read(self.channelA)
		if (self.prev_state_A != stateA):
			if (self.pi.read(self.channelB) != stateA):
				self.count -= 1
			else:
				self.count += 1

		self.prev_state_A = stateA

	'''def edge_detected(self, gpio, level, tick):

			state_A = self.pi.read(self.channelA)
			state_B = self.pi.read(self.channelB)
			if state_B != state_A:
				self.count -= 1
			else:
				self.count += 1'''

	def get_count(self):

		count = self.count
		return count

	def get_position(self, heading, counter):
		# with self.lock:
		# revolution = (self.prev_count - self.count)/1040

		revolution = (counter/2060)  #168
		distance_cm = revolution * self.const
		change = distance_cm - self.prev_distance
		self.dx = math.cos(math.radians(heading)) * change
		self.dy = math.sin(math.radians(heading)) * change
		# print(f"Change : {change}")

		self.x += self.dx
		self.y += self.dy

		error_x = self.x - math.cos(math.radians(heading)) * distance_cm
		error_y = self.y - math.sin(math.radians(heading)) * distance_cm

		self.error_x = error_x
		self.error_y = error_y
		# self.x_history.append(self.x)
		# self.y_history.append(self.y)
		self.prev_distance = distance_cm

		# self.prev_count = self.count
		return self.x, self.y

	'''def get_position(self, heading, counter, dx, dy):
		#with self.lock:
		#revolution = (self.prev_count - self.count)/1040

		revolution = (counter/168)
		distance_cm = revolution*self.const
		change = distance_cm - self.prev_distance
		dx = math.cos(math.radians(heading)) * change
		dx = math.sin(math.radians(heading)) * change
		#print(f"Change : {change}")
		#self.error_x += dx - math.cos(math.radians(heading - self.error_x)) * change
		#self.error_y += dy - math.sin(math.radians(heading - self.error_y)) * change
		self.x += dx 
		self.y += dx 
		#self.x_history.append(self.x)
		#self.y_history.append(self.y)
		self.prev_distance = distance_cm
			
			#self.prev_count = self.count
		return self.x, self.y'''

	def stop_counter(self):
		# with self.lock:
		self.cb_A.cancel()
		self.cb_B.cancel()
		self.pi.stop()  # Clean up pigpio on exit
		GPIO.cleanup()

	def plotPath(self, ax):
		# with self.lock:
		ax.cla()
		ax.plot(self.x_history, self.y_history)
		ax.set_xlabel('X cm')
		ax.set_ylabel('Y cm')
		ax.set_title('Robot Position')
		plt.show()
