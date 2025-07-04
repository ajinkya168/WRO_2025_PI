# -*- coding: utf-8 -*


import os

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
import pigpio
import time

RX_Head = 23
RX_Left = 24
RX_Right = 25
pi = pigpio.pi()
pi.set_mode(RX_Head, pigpio.INPUT)
pi.set_mode(RX_Left, pigpio.INPUT)
pi.set_mode(RX_Right, pigpio.INPUT)
pi.bb_serial_read_open(RX_Head, 115200)
pi.bb_serial_read_open(RX_Left, 115200)
pi.bb_serial_read_open(RX_Right, 115200)
distance_head = 0
distance_left = 0
distance_right = 0

prev_distance = 0


def getTFminiData():
	# while True:

	# print("#############")
	time.sleep(0.05)  # change the value if needed
	# (count, recv) = pi.bb_serial_read(RX)
	(count_head, recv_head) = pi.bb_serial_read(RX_Head)
	(count_left, recv_left) = pi.bb_serial_read(RX_Left)
	(count_right, recv_right) = pi.bb_serial_read(RX_Right)
	if count_head > 8:
		for i in range(0, count_head - 9):
			if recv_head[i] == 89 and recv_head[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_head[i + j]
				checksum = checksum % 256
				if checksum == recv_head[i + 8]:
					global distance_head
					distance_head = recv_head[i + 2] + recv_head[i + 3] * 256
					strength_head = recv_head[i + 4] + recv_head[i + 5] * 256
					# print("distance_head : ", distance_head)

	if count_left > 8:
		for i in range(0, count_left - 9):
			if recv_left[i] == 89 and recv_left[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_left[i + j]
				checksum = checksum % 256
				if checksum == recv_left[i + 8]:
					global distance_left
					distance_left = recv_left[i + 2] + recv_left[i + 3] * 256
					strength_left = recv_left[i + 4] + recv_left[i + 5] * 256
					# print("distance_left : ", distance_left)

	if count_right > 8:
		for i in range(0, count_right - 9):
			if recv_right[i] == 89 and recv_right[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_right[i + j]
				checksum = checksum % 256
				if checksum == recv_right[i + 8]:
					global distance_right
					distance_right = recv_right[i + 2] + recv_right[i + 3] * 256
					strength_right = recv_right[i + 4] + recv_right[i + 5] * 256
					# print("distance_right : ", distance_right)

	print(
		"distance_Head : {}, distance_left: {}, distance_right: {}".format(
			distance_head, distance_left, distance_right
		)
	)


if __name__ == "__main__":

	try:
		CMD_DISABLE_HEADING = bytearray([0x56, 0x02, 0x00])

		while True:
			getTFminiData()
			# time.sleep(1)
			# CMD_ENABLE_HEADING = bytearray([0x56, 0x02, 0x01])

			# Send the command to disable heading output

			# getTFminiData()
			'''print(
				"distance_Head : {}, distance_left: {}, distance_right: {}".format(
					distance_head, distance_left, distance_right
				)
			)'''
	except Exception as e:
		print(e)
		pi.bb_serial_read_close(RX_Head)
		pi.bb_serial_read_close(RX_Left)
		pi.bb_serial_read_close(RX_Right)
		pi.stop()
