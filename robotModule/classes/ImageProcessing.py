import cv2
import numpy as np
from picamera2 import Picamera2
from multiprocessing import Process, Value, Array


class LiveFeed:
	def __init__(self, color_b, stop_b, red_b, green_b, pink_b, centr_y):
		self.color_b = color_b
		self.stop_b = stop_b
		self.red_b = red_b
		self.green_b = green_b
		self.pink_b = pink_b
		self.centr_y = centr_y
		self.picam2 = Picamera2()
		self.kernel = np.ones((3, 3), 'uint8')

    def BrightnessContrast(self, brightness=0):
        # getTrackbarPos returns the
        # current position of the specified trackbar.
        brightness = cv2.getTrackbarPos('Brightness', 'GEEK')

        contrast = cv2.getTrackbarPos('Contrast', 'GEEK')

        effect = controller(img, brightness, contrast)

        # The function imshow displays
        # an image in the specified window
        cv2.imshow('Effect', effect)
	def run(self):
		print('Image Process started')
		self.picam2.preview_configuration.main.size = (1280, 720)
		self.picam2.preview_configuration.main.format = 'RGB888'
		self.picam2.preview_configuration.align()
		self.picam2.configure('preview')
		self.picam2.start()

		cv2.namedWindow('Object Dist Measure ', cv2.WINDOW_NORMAL)
		cv2.resizeWindow('Object Dist Measure ', 1280, 720)

		while True:
			img = self.picam2.capture_array()

			hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
			hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
			hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink

			# For Green Color
			lower = np.array([40, 54, 36])  # green
			upper = np.array([73, 150, 155])
			mask = cv2.inRange(hsv_img, lower, upper)
			mask = cv2.dilate(mask, kernel, iterations=3)
			mask = cv2.erode(mask, kernel, iterations=3)

			# For Red Color
			lower1 = np.array([171, 159, 61])  # red
			upper1 = np.array([179, 255, 147])
			mask1 = cv2.inRange(hsv_img1, lower1, upper1)
			mask1 = cv2.dilate(mask1, kernel, iterations=5)
			mask1 = cv2.erode(mask1, kernel, iterations=5)

			# For Pink Color
			lower2 = np.array([154, 168, 83])  # pink
			upper2 = np.array([171, 217, 171])
			mask2 = cv2.inRange(hsv_img2, lower2, upper2)
			mask2 = cv2.dilate(mask2, kernel, iterations=5)
			mask2 = cv2.erode(mask2, kernel, iterations=5)

			# Remove Extra garbage from image
			d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=5)  # green
			d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel, iterations=5)  # red
			d_img2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel, iterations=5)  # pink

			# find the histogram
			cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

			cont1, hei1 = cv2.findContours(d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

			cont2, hei2 = cv2.findContours(d_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cont2 = sorted(cont2, key=cv2.contourArea, reverse=True)[:1]

			# ----------------------------------------------
			if len(cont) == 0:
				green_present = False
			else:
				max_cnt = max(cont, key=cv2.contourArea)
				green_present = True
			# ---------------------------------------
			if len(cont1) == 0:
				red_present = False

			else:
				max_cnt1 = max(cont1, key=cv2.contourArea)
				red_present = True
			# ----------------------------------------------------
			if len(cont2) == 0:
				pink_present = False

			else:
				max_cnt2 = max(cont2, key=cv2.contourArea)
				pink_present = True
			# --------------------------------------------------------

			if not red_present and not green_present and not pink_present:
				color_b.value = False
				red_b.value = False
				green_b.value = False
				pink_b.value = False
				pink_present = False

			if (red_present and green_present and pink_present):
				all_flag = True
				both_flag = False
				pink_red = False
				pink_green = False
			if red_present and green_present and not pink_present:
				both_flag = True
				all_flag = False
				only_red = False
				only_green = False
				only_pink = False
				pink_red = False
				pink_green = False
			if red_present and not pink_present and not green_present:
				only_red = True
				both_flag = False
				all_flag = False
				only_green = False
				only_pink = False
				pink_red = False
				pink_green = False
			if green_present and not pink_present and not red_present:
				only_green = True
				both_flag = False
				all_flag = False
				only_red = False
				only_pink = False
				pink_red = False
				pink_green = False
			if pink_present and not green_present and not red_present:
				only_pink = True
				both_flag = False
				all_flag = False
				only_red = False
				only_green = False
				pink_red = False
				pink_green = False
			if pink_present and green_present and not red_present:
				only_pink = True
				both_flag = False
				all_flag = False
				only_red = False
				only_green = False
				pink_red = False
				pink_green = True

			if pink_present and red_present and not green_present:
				only_pink = True
				both_flag = False
				all_flag = False
				only_red = False
				only_green = False
				pink_red = True
				pink_green = False
			if all_flag:
				color_b.value = True
				print("ITS THE FIRST LOOP")
				### FOR GREEN BOX
				if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1) and cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
					if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
						# Draw a rectange on the contour
						rect = cv2.minAreaRect(max_cnt)
						box = cv2.boxPoints(rect)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
						(x, y, w, h) = cv2.boundingRect(box)
						centroid_y = y + h // 2
						centr_y.value = centroid_y
						green_b.value = True
						red_b.value = False
						pink_b.value = False

				### FOR RED BOX
				if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
					if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
						# Draw a rectange on the contour
						rect1 = cv2.minAreaRect(max_cnt1)
						box = cv2.boxPoints(rect1)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = True
						green_b.value = False
						pink_b.value = False

				#### FOR PINK BOX
				if cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
					if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
						# Draw a rectange on the contour
						rect2 = cv2.minAreaRect(max_cnt2)
						box = cv2.boxPoints(rect2)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = False
						green_b.value = False
						pink_b.value = True

            ### FOR RED BOX
			elif only_red:
				color_b.value = True
				# print(cv2.contourArea(max_cnt1))
				if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2
					centr_y.value = centroid_y
					red_b.value = True
					pink_b.value = False
					green_b.value = False


			### FOR GREEN BOX
			elif only_green:
				color_b.value = True
				# print(cv2.contourArea(max_cnt))
				if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2
					centr_y.value = centroid_y
					# if(centroid_y > 100):
					# if(counter_green >= max_count):
					green_b.value = True
					pink_b.value = False
					red_b.value = False
					counter_red = 0

			### FOR PINK BOX
			elif only_pink:
				if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
					# Draw a rectange on the contour
					rect2 = cv2.minAreaRect(max_cnt2)
					box = cv2.boxPoints(rect2)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2

					centr_y.value = centroid_y
					red_b.value = False
					green_b.value = False
					pink_b.value = True
			elif both_flag:
				color_b.value = True
				print("BOTH ARE PRESENT...")
				### FOR GREEN BOX
				if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
					if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
						# Draw a rectange on the contour
						rect = cv2.minAreaRect(max_cnt)
						box = cv2.boxPoints(rect)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
						(x, y, w, h) = cv2.boundingRect(box)
						centroid_y = y + h // 2
						centr_y.value = centroid_y
						green_b.value = True
						red_b.value = False
						pink_b.value = False

				### FOR RED BOX
				elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
					if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
						# Draw a rectange on the contour
						rect1 = cv2.minAreaRect(max_cnt1)
						box = cv2.boxPoints(rect1)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = True
						green_b.value = False
						pink_b.value = False

			elif pink_red:
				### FOR RED BOX
				if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
					if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
						# Draw a rectange on the contour
						rect1 = cv2.minAreaRect(max_cnt1)
						box = cv2.boxPoints(rect1)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = True
						green_b.value = False
						pink_b.value = False
				elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
					if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
						# Draw a rectange on the contour
						rect2 = cv2.minAreaRect(max_cnt2)
						box = cv2.boxPoints(rect2)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = False
						green_b.value = False
						pink_b.value = True
			elif pink_green:
				if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
					if (cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000):
						# Draw a rectange on the contour
						rect = cv2.minAreaRect(max_cnt)
						box = cv2.boxPoints(rect)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

						(x, y, w, h) = cv2.boundingRect(box)

						centroid_y = y + h // 2

						centr_y.value = centroid_y
						red_b.value = False
						green_b.value = True
						pink_b.value = False
				elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
					if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
						# Draw a rectange on the contour
						rect2 = cv2.minAreaRect(max_cnt2)
						box = cv2.boxPoints(rect2)
						box = np.intp(box)
						cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
						(x, y, w, h) = cv2.boundingRect(box)
						centroid_y = y + h // 2
						centr_y.value = centroid_y
						red_b.value = False
						green_b.value = False
						pink_b.value = True
			print(f"Green:{green_present}, red:{red_present}, pink:{pink_present}")
			cv2.imshow('Object Dist Measure ', img)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				self.stop_b.value = True
				break

		cv2.destroyAllWindows()
		self.picam2.stop()


if __name__ == '__main__':
	color_b = Value('b', False)
	stop_b = Value('b', False)
	red_b = Value('b', False)
	green_b = Value('b', False)
	pink_b = Value('b', False)
	centr_y = Array('i', [0])

	live_feed = LiveFeed(color_b, stop_b, red_b, green_b, pink_b, centr_y)

	p = Process(target=live_feed.run)
	p.start()
	# ... (other processes or main program logic)
	p.join()
