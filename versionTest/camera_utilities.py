def process_box(cnt, img, color=(255, 0, 0)):
    if cnt is not None:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        cv2.drawContours(img, [box], -1, color, 3)
        x, y, w, h = cv2.boundingRect(box)
        return x + w // 2, y + h // 2
    return 0, 0

def clean_mask(mask, kernel, iterations=2):
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    return closed


def get_largest_contour(binary_img):
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        return sorted(contours, key=cv2.contourArea, reverse=True)[:1]
    return []


def is_color_present(contours, min_area=2000):
    if not contours:
        return False, None
    max_c = max(contours, key=cv2.contourArea)
    return (cv2.contourArea(max_c) > min_area), max_c