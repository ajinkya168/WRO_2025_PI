from Servo import Servo

servo = Servo(8)

kp = 0.6
ki = 0
kd = 0.1

kp_e = 3  # 12
ki_e = 0
kd_e = 40  # 40if

corr = corr_pos = 0


currentAngle = error_gyro = prevError = totalError = prevErrorGyro = totalErrorGyro = correction =  0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0

def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x, finish, distance_h, distance_l, distance_r):
    # print("INSIDE CORRECT")
    global prevError, totalError, prevErrorGyro, totalErrorGyro, corr_pos

    error = correction = pTerm_e = dTerm_e = iTerm_e =0

    lane = counter % 4

    # if(time.time() - last_time > 0.001):
    if lane == 0:
        error = setPoint - y
        print(
            f"lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print(
                f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

        elif blue:
            error = (100 + setPoint) - x
            # print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
    elif lane == 2:
        if orange:
            error = y - (200 - setPoint)
            # print(f"lane:{lane} error:{error} target:{(200 - setPoint)},  x: {x} y{y}")
        elif blue:
            error = y - (-200 - setPoint)
            # print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
    elif lane == 3:
        if orange:
            error = (setPoint - 100) - x
            # print(f"lane:{lane} error:{error} target:{(setPoint - 100)}, x: {x} y {y}")

        elif blue:
            error = x + (100 + setPoint)
            # print(f"lane:{lane} error:{error} target:{(100 + setPoint)}, x:{x} y {y}")

    corr_pos = error
    pTerm_e = kp_e * error
    dTerm_e = kd_e * (error - prevError)
    totalError += error
    iTerm_e = ki_e * totalError
    correction = pTerm_e + iTerm_e + dTerm_e

    if setPoint == 0:
        if abs(error) < 10:
            # print("absolute is 0")
            correction = 0

    if not reset:
        if ((setPoint == -35 and orange) or (counter == 0 and (centr_x < 800 and centr_x > 0) and not blue and not orange) and not finish):
            if distance_l <= 30:
                correction = 20
                print(f"Avoiding pink wall {correction}")

            elif distance_r < 50:
                if distance_r <= 35:
                    correction = -45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = -10
                    print(f"Avoiding pink wall {correction}")

            else:
                correction = 0

        elif ((setPoint == 35 and blue) or (counter == 0 and (centr_x < 800 and centr_x > 0)  and not blue and not orange) and not finish):

            if distance_r <= 30:
                correction = -20
                print(f"Avoiding pink wall {correction}")

            elif distance_l < 50 or distance_l > 100:
                if distance_l <= 35:
                    correction = 45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = 10
                    print(f"Avoiding pink wall {correction}")
            else:
                correction = 0

        if not blue:
            if (setPoint <= -70) and distance_l <= 22:
                print(f"Correcting Green Wall Orange")
                correction = 10
            else:
                pass

            if setPoint >= 70 and (distance_r <= 20 or (distance_h <= 18)):
                print(f"Wall detected..making correction")
                correction = -10
            else:
                pass

        else:
            if setPoint <= -70 and (distance_l <= 20 or (distance_h <= 18)):
                print(f"Wall detected..making correction")
                correction = 10
            else:
                pass

            if setPoint >= 70 and distance_r <= 22:
                print(f"correctng red wall in blue")
                correction = -10
            else:
                pass

    if setPoint == 0:
        if correction > 25:
            correction = 25
        elif correction < -25:
            correction = -25
    else:
        if correction > 45:
            correction = 45
        elif correction < -45:
            correction = -45

    prevError = error
    correctAngle(head + correction, heading)


def correctAngle(setPoint_gyro, heading):
    #global corr
    error_gyro = prevErrorGyro = totalErrorGyro = correction  = 0

    error_gyro = heading - setPoint_gyro

    if error_gyro > 180:
        error_gyro = error_gyro - 360
    corr = error_gyro
    # print("Error : ", error_gyro)
    pTerm = dTerm = iTerm =  0


    pTerm = kp * error_gyro
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if correction > 30:
        correction = 30
    elif correction < -30:
        correction = -30

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)
