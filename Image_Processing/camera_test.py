import cv2

# Open camera
cap = cv2.VideoCapture(0)

# Try to set manual exposure and white balance
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)       # 0.25 = manual mode (for many UVC cameras)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)              # Try values from -1 to -13 (depends on camera)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)                # 0 = turn off auto white balance
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4650)      # Tune between 2800 (warm) to 6500 (cool)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Camera Feed", frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
