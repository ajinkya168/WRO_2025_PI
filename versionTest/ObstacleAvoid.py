
class ObstacleAvoid:


    def __int__(self, tfmini, servo):
        self.kp = 0.6
        self.ki = 0
        self.kd = 0.1
        self.tfmini = tfmini
        self.ser = servo

        self.kp_e = 3 # 12
        self.ki_e = 0
        self.kd_e = 40 #40


        self.corr = 0
        self.totalError = 0
        self.prevError = 0
        self.prevErrorGyro = 0
        self.totalErrorGyro = 0
    def correctPosition(self, setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x, finish):
        # print("INSIDE CORRECT")
        #self.tfmini.getTFminiData()
        error = 0
        correction = 0
        pTerm_e = 0
        dTerm_e = 0
        iTerm_e = 0
        lane = counter % 4
        # if(time.time() - last_time > 0.001):
        if lane == 0:
            if not reverse:
                if orange:
                    error = setPoint - y
                elif blue:
                    error = setPoint + y
                else:
                    error = setPoint - y
                print(f" 1 lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse" )

            elif reverse:
                if blue:
                    error = setPoint - y
                elif orange:
                    error = setPoint - y
                print(f"2 lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse" )

        # print(f"lane:{lane} error: {error} tagret:{setPoint}")
        # print(f"trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        elif lane == 1:
            if orange:
                error = x - (100 - setPoint)
                print(f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

            elif blue:
                error = (100 + setPoint) - x
                print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
        # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        elif lane == 2:
            if orange:
                error = y - (200 - setPoint)
                print(f"lane:{lane} error:{error} target:{(200 - setPoint)},  x: {x} y{y}")
            elif blue:
                error = y - (-200 - setPoint)
                print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
        # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        elif lane == 3:
            if orange:
                error = (setPoint - 100) - x
                print(f"lane:{lane} error:{error} target:{(setPoint - 100)}, x: {x} y {y}")

            elif blue:
                error = x + (100 + setPoint)
                print(f"lane:{lane} error:{error} target:{(100 + setPoint)}, x:{x} y {y}")

        # print(f"lane:{lane} error: {error} target:{-100 - setPoint}")
        # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        # last_time = time.time()

        corr_pos = error
        pTerm_e = self.kp_e * error
        dTerm_e = self.kd_e * (error - self.prevError)
        self.totalError += error
        iTerm_e = self.ki_e * self.totalError
        correction = pTerm_e + iTerm_e + dTerm_e

        # print("correction: {}, x:{}, y:{}, heading:{} ".format(correction, x, y, glob))

        # if (setPoint_flag == 0)

        if setPoint == 0:
            if abs(error) < 10:
                print("absolute is 0")
                correction = 0

        if setPoint == -35 and orange:
            self.tfmini.getTFminiData()
            if self.tfmini.distance_right < 55 and (centr_x < 600 and centr_x > 0) and not finish:
                print(f"correcting pink wall...")
                correction = -40
            else:
                pass

        elif setPoint == 35 and blue:
            self.tfmini.getTFminiData()
            if self.tfmini.distance_left < 55 and (centr_x > 600 and centr_x > 0) and not finish:
                print("correcting pink wall blue")
                correction = 40
            else:
                pass

        # print("In the  correct Position")

        if not reset:
            self.tfmini.getTFminiData()

            if not blue:
                if (setPoint <= -70) and self.tfmini.distance_left <= 25:
                    print(f"Correcting Green Wall Orange")
                    correction = 10
                else:
                    pass

                if setPoint >= 70 and (self.tfmini.distance_right < 25 or (self.tfmini.distance_head < 20)):
                    print(f"Wall detected..making correction")
                    correction = -10
                else:
                    pass

            else:
                if setPoint <= -70 and (self.tfmini.distance_left < 25 or (self.tfmini.distance_head < 20)):
                    print(f"Wall detected..making correction")
                    correction = 10
                else:
                    pass
                if setPoint >= 70 and self.tfmini.distance_right < 25:
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

        # print(f"Correction in position:{correction}")

        # print(f"lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        # print("correction: ", correction)

        prevError = error
        #print(f"Correction: {head - correction}")
        self.correctAngle(head + correction, heading)
    def correctAngle(self, setPoint_gyro, imu_heading):

        error_gyro = imu_heading - setPoint_gyro

        if error_gyro > 180:
            error_gyro = error_gyro - 360

        self.corr = error_gyro


        pTerm = self.kp * error_gyro
        dTerm = self.kd * (error_gyro - self.prevErrorGyro)
        self.totalErrorGyro += error_gyro
        iTerm = self.ki * self.totalErrorGyro
        correction = pTerm + iTerm + dTerm

        if correction > 30:
            correction = 30
        elif correction < -30:
            correction = -30

        self.prevErrorGyro = error_gyro
        self.servo.setAngle(90 - correction)