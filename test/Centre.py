import cv2
import numpy as np

# Initialize the camera
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280,720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

def Centre(frame):
	height, width, _ = frame.shape

	# Calculate the center coordinates
	center_x = width // 2
	center_y = height // 2
	# Define the cube vertices (8 points)
	cube_size = 5
	vertices = np.array([
		[center_x - cube_size, center_y - cube_size],
		[center_x + cube_size, center_y - cube_size],
		[center_x + cube_size, center_y + cube_size],
		[center_x - cube_size, center_y + cube_size],
		[center_x - cube_size, center_y - cube_size],
		[center_x + cube_size, center_y - cube_size],
		[center_x + cube_size, center_y + cube_size],
		[center_x - cube_size, center_y + cube_size]
	])

	# Define the cube edges (12 lines)
	edges = np.array([
		[0, 1],
		[1, 2],
		[2, 3],
		[3, 0],
		[4, 5],
		[5, 6],
		[6, 7],
		[7, 4],
		[0, 4],
		[1, 5],
		[2, 6],
		[3, 7]
	])

	# Draw the cube
	for edge in edges:
		cv2.line(frame, tuple(vertices[edge[0]]), tuple(vertices[edge[1]]), (0, 255, 0), 1)
		
	return frame
    

while True:
    # Read a frame from the camera
    frame = picam2.capture_array()
    
    ctr = Centre(frame)
    # Display the frame
    cv2.imshow('Cube', ctr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
picam2.stop()
cv2.destroyAllWindows()
