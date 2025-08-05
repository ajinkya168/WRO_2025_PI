class DetectorResult:
    def __init__(self, detector_data):
        self.class_name = detector_data["class"]
        self.class_id = detector_data["classID"]
        self.confidence = detector_data["conf"]
        self.points = detector_data["pts"]  # [[x1,y1], [x2,y2], ...]
        self.target_area = detector_data["ta"]
        self.target_x_degrees = detector_data["tx"]
        self.target_x_pixels = detector_data["txp"]
        self.target_y_degrees = detector_data["ty"]
        self.target_y_pixels = detector_data["typ"]