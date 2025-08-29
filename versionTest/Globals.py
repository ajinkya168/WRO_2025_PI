import multiprocessing


rplidar = [None]*360
previous_distance = 0
dist_0 = 0
dist_90 = 0
dist_270 = 0
angle = 0
lidar_front = 0
lidar_left = 0
lidar_right = 0


############ PID VARIABLES #############

currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
prevError = 0
totalError = 0
correcion = 0

kp = 0.6
ki = 0
kd = 0.1

kp_e = 3  # 12
ki_e = 0
kd_e = 40  # 40if

corr = 0
corr_pos = 0

###################################################


RX_Head = 23
RX_Left = 24
RX_Right = 25
RX_Back = 27
button_pin = 5
servo_pin = 8
blue_led = 26
red_led = 10
green_led = 6
reset_pin = 19

############# FLAGS ###############
button = False

trigger = reset_f = False
blue_flag = False
orange_flag = False

change_path = False
timer_started = False
timer_v = 0
g_flag = r_flag = p_flag = False
g_past = r_past = p_past = False

red_stored = green_store = False

red_turn = green_turn = False
calc_time = False
lap_finish = continue_parking = parking_heading = parking_flag = False
turn_flag = reset_flags = counter_reset = False
finished = finish = stop_flag = False
red_time = green_time = False
pink_detected = False
last_red = False
cw = ccw = False
reset_heading = previous_heading_stored = False
pink_timer = pink_r = False
back_bot = False
green_timer = red_timer = False
g_last_flag = r_last_flag = False
reverse_complete = reverse = reverse_trigger = False
blue_on = False
finish_flag = False
reset_servo = False
trigger_reset = False
not_block = False

############ VARIABLES ##################
color_n = ""
setPointL = -70
setPointR = 70
setPointC = 0
power = 95
prev_power = 0
last_counter = 12
change_counter = 7  # 3
rev_counter = 7
counter = turn_t = current_time = gp_time = rp_time = buff = c_time = green_count = red_count = 0
heading_angle = 0
i = l = lap_finish_time = prev_distance = turn_trigger_distance = target_count = offset = button_state = past_time = 0

previous_heading = -1
stop_time = turn_cos_theta = parking_done = pink_d = g_time = r_time = u = avg_right = avg_head = avg_left = 0

time_p = prev_time = prev_restore = finish_timer = prev_blue = prev_orange = avg_blue = avg_orange = 0

c = c_time = fps_time2 = 0

color_s = ""
debounce_delay = 0.2
last_time = 0
avoided = False
avoided_time = time.time()
reverse_until = 0
encoder_counter_store = False
encoder_counts_value = 0
l_left = 0
off = 5000
rev_count = 0
reverse_true = False
parking_flag = False
parking_heading_reverse = False
parking_rev_count = 0
trigger_enc_flag = False
trigger_enc = 0
inParkingatStart = False
startPark = 0
exitPark = False
timer = 0
norm_head = 0
lane_reset = 0
red_time = 0
green_time = 0
pink_time = 0
