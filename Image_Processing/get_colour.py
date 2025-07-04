import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')

import numpy as np
import cv2
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1600,1000)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()




def empty(a):
    pass


def resize_final_img(x,y,*argv):
    images  = cv2.resize(argv[0], (x, y))
    for i in argv[1:]:
        resize = cv2.resize(i, (x, y))
        images = np.concatenate((images,resize),axis = 1)
    return images

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 300, 300)
cv2.createTrackbar("HUE Min", "HSV", 0, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 179, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 0, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)
cv2.createTrackbar("R1 Max", "HSV", 179, 179, empty)
cv2.createTrackbar("R1 Min", "HSV", 0, 179, empty)


cv2.resizeWindow('F', 700,600)


while True:
    img= picam2.capture_array()
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
    r1_max = cv2.getTrackbarPos("R1 Max", "HSV")
    r1_min = cv2.getTrackbarPos("R1 Min", "HSV")
   
    
	
    #blurred = cv2.medianBlur(img, 15)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    r1upper = np.array([r1_max, s_max, v_max])
    r1lower = np.array([r1_min, s_min, v_min])
    mask1 = cv2.inRange(hsv_img, lower, upper)
    #mask2 = cv2.inRange(hsv_img, r1lower, r1upper)
    #mask = mask1 + mask2
    mask = mask1
    kernel = np.ones((3,3),'uint8')
   #mask = cv2.
    #mask = cv2.dilate(mask, kernel, iterations=1)
    #mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,iterations = 2)
    d_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,iterations = 2)

    

    final_img = resize_final_img(300,300, d_img)
    # final_img = np.concatenate((mask,d_img,e_img),axis =1)
    cv2.imshow('F',final_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
