from drive_controller import DrivingController


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True
        self.collision_flag = True
        self.moveCenter = 0
        self.reverseTrigger = 25

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        ###########################################################################

        # Default commands
        setSteering = 0
        toMiddle = sensing_info.to_middle
        car_controls.brake = 0

        # Calculate each angle of track independently
        forwardAngles = [sensing_info.track_forward_angles[0]]
        for i in range(1, 10):
            forwardAngles.append(sensing_info.track_forward_angles[i]-sensing_info.track_forward_angles[i-1])

        # calculate the center based on the curve ahead
        curveCenter = self.curve_center(sensing_info, forwardAngles)[0]
        finalCurveCenter = self.curve_center(sensing_info, forwardAngles)[1]

        # create your own list of obstacles ahead of you
        obstacleList = []
        for obstacle in sensing_info.track_forward_obstacles:
            if obstacle['dist'] >= 0:
                if obstacle['to_middle'] <= (self.half_road_limit+1):
                    obstacleList.append(obstacle)
        # if there is an obstacle calculate the optimal center
        if len(obstacleList) > 0:
            obstacleCenter = self.obstacle_center(sensing_info, obstacleList, curveCenter, finalCurveCenter)
            calculatedCenter = self.calculate_obstacle_center(sensing_info, curveCenter, obstacleCenter, obstacleList)
        # otherwise the center is just the center according to the curve
        else:
            calculatedCenter = curveCenter

        # based on where the center zone is calculate how far you are from it
        toMiddle -= calculatedCenter
        # based on how far you are from the middle calculate what zone you are in
        zone = self.calculate_zone(toMiddle)
        # predictive angle so that the car turns before the curve
        angle = sensing_info.moving_angle - (forwardAngles[0]+forwardAngles[1])/2
        # calculate steering based on the zone and the angle of the car to the road
        car_controls.steering = self.calculate_steering(zone, angle, forwardAngles)

        # If going too fast or at an extreme angle, slow down
        car_controls.throttle = self.speedlimit(sensing_info, zone, forwardAngles, angle)[0]

        # If collided and car has slowed to a stop or is going backwards, reverse, stop, and then accelerate
        if sensing_info.collided == True:
            if sensing_info.speed < 10 and sensing_info.lap_progress > 1.5:
                self.reverseTrigger -= 1
        if self.reverseTrigger < 25 and self.reverseTrigger >= 20:
            car_controls.throttle = 0
            car_controls.steering = 0
            self.reverseTrigger -= 1
        elif self.reverseTrigger < 20 and self.reverseTrigger >= 10:
            car_controls.throttle = -1
            car_controls.steering = 0
            self.reverseTrigger -= 1
        elif self.reverseTrigger < 10:
            car_controls.throttle = 0
            car_controls.steering = 0
            car_controls.brake = 1
            self.reverseTrigger -= 1
        if self.reverseTrigger < 0:
            self.reverseTrigger = 25


        # ==========================================================#
        print( "to middle: {:5}".format(sensing_info.to_middle),
        "|limit: {:3}".format(self.speedlimit(sensing_info, zone, forwardAngles, angle)[1]),
        "|speed: {:6}".format(sensing_info.speed),
        "|angle: {:5}".format(sensing_info.moving_angle),
        # "|calculated angle: {:5}".format(round(angle,1)),
        "|curve center: {:5}".format(round(curveCenter,2)),
        "|calculated center: {:5}".format(round(calculatedCenter,2)),
        # "|reverseTrigger: {:3}".format(self.reverseTrigger),
        "|steering: {:4}".format(car_controls.steering),
        "|throttle: {:4}".format(car_controls.throttle),
        # "|brake: {}".format(car_controls.brake),
        # "|obstacles: ", sensing_info.track_forward_obstacles,
        "|forward angles: {}".format(forwardAngles),
        "|lap progress: {}".format(sensing_info.lap_progress))

        return car_controls


    #######################################
    # CUSTOM FUNCTIONS
    #######################################

    # calculate the center given the optimal obstacle center and the optimal curve center
    def calculate_obstacle_center(self, sensing_info, curveCenter, obstacleCenter, obstacleList):
        calculatedCenter = curveCenter
        distObs = obstacleList[0]['dist']
        if distObs < 70:
            if distObs < 70 and distObs >= 60:
                calculatedCenter = curveCenter - (curveCenter-obstacleCenter)/3
            if distObs < 60 and distObs >= 50:
                calculatedCenter = curveCenter - (curveCenter-obstacleCenter)*2/3
            elif distObs < 50:
                calculatedCenter = obstacleCenter
        return calculatedCenter

    # caculate the center given a second player and a curve center
    def calculate_p2_center(self, sensing_info, curveCenter, P2center):
        calculatedCenter = curveCenter
        distP2 = sensing_info.opponent_cars_info[0]['dist']
        if distP2 < 50:
            if distP2 < 50 and distP2 >= 40:
                calculatedCenter = curveCenter - (curveCenter-P2center)/3
            elif distP2 < 40 and distP2 >= 30:
                calculatedCenter = curveCenter - (curveCenter-P2center)*2/3
            elif distP2 < 30:
                calculatedCenter = P2center
        return calculatedCenter

    # calculate the appropriate center based on the curve ahead
    def curve_center(self, sensing_info, forward_angles):
        curveCenter = 0
        finalCurveCenter = 0
        for i in range(4,7):
            if forward_angles[i] > 20:
                curveCenter -= (self.half_road_limit)/4
                finalCurveCenter = -(self.half_road_limit*3)/4
            if forward_angles[i] < -20:
                curveCenter += (self.half_road_limit)/4
                finalCurveCenter = (self.half_road_limit*3)/4
        for i in range(0,3):
            if forward_angles[i] > 10:
                curveCenter += (self.half_road_limit)/4
                finalCurveCenter = (self.half_road_limit*3)/4
            if forward_angles[i] < -10:
                curveCenter -= (self.half_road_limit)/4
                finalCurveCenter = -(self.half_road_limit*3)/4
        return (curveCenter, finalCurveCenter)

    # based on obstacles calculate the optimal obstacle center
    # DOES NOT WORK FOR MARINA BAY // wrong logic somewhere
    def obstacle_center(self, sensing_info, obstacleList, curveCenter, finalCurveCenter):
        obstacleBlock = False
        obstacleCenter = curveCenter
        for obstacle in obstacleList:
            if finalCurveCenter > obstacle['to_middle'] - 4.5 and finalCurveCenter < obstacle['to_middle'] + 4.5:
                obstacleBlock = True
        if obstacleBlock == True:
            ## ONLY GET THE OBSTACLES THAT MATTER IMMEDIATELY
            obstacleList = sorted(obstacleList,key=lambda x: x['dist'])
            relevantObstacles = [obstacleList[0]]
            closestObstacle = obstacleList[0]['dist']
            for obstacle in obstacleList:
                if obstacle['dist'] - closestObstacle < 20:
                    relevantObstacles.append(obstacle)
            ## BASED ON WHICH WAY YOU'RE TURNING, FIND THE PATH
            obstacleCenters = []
            goodCenters = []

            for obstacle in relevantObstacles:
                obstacleCenters.append(obstacle['to_middle']-5)
                obstacleCenters.append(obstacle['to_middle']+5)
            for center in obstacleCenters:
                goodCenter = True
                for obstacle in relevantObstacles:
                    if center > obstacle['to_middle'] - 4.5 and center < obstacle['to_middle'] + 4.5:
                        goodCenter = False
                    if center > self.half_road_limit-1 or center < -self.half_road_limit+1:
                        goodCenter = False
                if goodCenter == True:
                    goodCenters.append(center)
            if len(goodCenters) > 0:
                closestCenter = min(goodCenters, key=lambda x:abs(x-sensing_info.to_middle))
                obstacleCenter = closestCenter
        return obstacleCenter

    # if an opponent is ahead, calculate a center that avoids the opponent
    def opponent_center(self, sensing_info):
        distP2 = sensing_info.opponent_cars_info[0]['dist']
        toMiddleP2 = sensing_info.opponent_cars_info[0]['to_middle']
        speedP2 = sensing_info.opponent_cars_info[0]['speed']
        P2center = 0
        # Player 2 has entered the game
        if toMiddleP2 >= 0:
            P2center = toMiddleP2 - 5
        elif toMiddleP2 < 0:
            P2center = toMiddleP2 + 5
        return P2center

    # if there is an opponent and obstacles, define how to avoid it
    def obstacle_opponent_center(self, sensing_info, obstacleList):
        distP2 = sensing_info.opponent_cars_info[0]['dist']
        toMiddleP2 = sensing_info.opponent_cars_info[0]['to_middle']
        speedP2 = sensing_info.opponent_cars_info[0]['speed']
        obsToMiddle = obstacleList[0]['to_middle']
        if obsToMiddle > 0 and toMiddleP2 > 0:
            combinedCenter = min(obsToMiddle, toMiddleP2) - 5
        elif obsToMiddle < 0 and toMiddleP2 < 0:
            combinedCenter = max(obsToMiddle, toMiddleP2) + 5
        else:
            if abs(obsToMiddle) < abs(toMiddleP2):
                if obsToMiddle < 0:
                    combinedCenter = obsToMiddle - 5
                else:
                    combinedCenter = obsToMiddle + 5
            if abs(toMiddleP2) < abs(obsToMiddle):
                if toMiddleP2 < 0:
                    combinedCenter = toMiddleP2 - 5
                else:
                    combinedCenter = toMiddleP2 + 5
        return combinedCenter

    # calculate zone based on distance from middle
    def calculate_zone(self, toMiddle):
        zone = 0
        if toMiddle < -(self.half_road_limit/2): zone = 1
        elif toMiddle < -(self.half_road_limit/4): zone = 2
        elif toMiddle <= (self.half_road_limit/4): zone = 3
        elif toMiddle <= (self.half_road_limit/2): zone = 4
        elif toMiddle > (self.half_road_limit/2): zone = 5
        return zone

    # calculate steering based on zone, angle of car, and severity of curves ahead
    def calculate_steering(self, zone, angle, forwardAngles):
        severity = 0
        steeringCap = 0.3
        range1 = -3
        range2 = 4

        angles = list(map(abs, forwardAngles[0:2]))
        if max(angles) >= 25:
            severity = 4
        if max(angles) >= 20 and max(angles) < 25:
            severity = 3
        if max(angles) >= 15 and max(angles) < 20:
            severity = 2
        if max(angles) >= 10 and max(angles) < 10:
            severity = 1

        if severity == 1:
            steeringCap = 0.4
            range1 = -4
            range2 = 5
        if severity == 2:
            steeringCap = 0.5
            range1 = -5
            range2 = 6
        if severity ==  3:
            steeringCap = 0.6
            range1 = -6
            range2 = 7
        if severity == 4:
            steeringCap = 0.8
            range1 = -8
            range2 = 9

        setSteering = 0
        if zone == 1:
            setSteering = steeringCap
            for i in range(range1,0):
                if angle > (i*3)+10:
                    setSteering -= 0.1
            for i in range(1,range2):
                if angle > (i*3)+10:
                    setSteering -= 0.1
        if zone == 2:
            setSteering = steeringCap
            for i in range(range1,0):
                if angle > (i*3)+5:
                    setSteering -= 0.1
            for i in range(1, range2):
                if angle > (i*3)+5:
                    setSteering -= 0.1
        if zone == 3:
            setSteering = steeringCap
            for i in range(range1,0):
                if angle > i*3:
                    setSteering -= 0.1
            for i in range(1, range2):
                if angle > i*3:
                    setSteering -= 0.1
        if zone == 4:
            setSteering = steeringCap
            for i in range(range1,0):
                if angle > (i*3)-5:
                    setSteering -= 0.1
            for i in range(1, range2):
                if angle > (i*3)-5:
                    setSteering -= 0.1
        if zone == 5:
            setSteering = steeringCap
            for i in range(range1,0):
                if angle > (i*3)-10:
                    setSteering -= 0.1
            for i in range(1, range2):
                if angle > (i*3)-10:
                    setSteering -= 0.1
        return round(setSteering, 1)

    # Control speed
    def control(self, sensing_info, zone, forwardAngles, angle):
        throttle = 1
        straightRoad = True
        carStable = True
        noObstacles = True
        speedlimit = 90
        for i in (7,10):
            if forwardAngles[i] > 5 or forwardAngles[i] < -5:
                straightRoad = False
                break
        if sensing_info.moving_angle > 5 or sensing_info.moving_angle < -5:
            carStable = False
        if zone != 3:
            carStable = False
        if len(sensing_info.track_forward_obstacles) > 0:
            if sensing_info.track_forward_obstacles[0]['dist'] < 30:
                noObstacles = False
        if straightRoad and carStable and noObstacles:
            speedlimit = 130
        if sensing_info.speed > speedlimit:
            throttle = 0.2
        if angle > 30 or angle < -30:
            throttle = 0.2
        if sensing_info.speed < 30:
            throttle = 1
        if abs(sensing_info.to_middle) > self.half_road_limit:
            throttle = 1
        return throttle

    # Speed limit setting
    def speedlimit(self, sensing_info, zone, forwardAngles, angle):
        speedlimit = 130
        throttle = 1.0
        angles = list(map(abs, forwardAngles[4:9]))
        if max(angles) >= 25:
            speedlimit = 80
        if max(angles) >= 20 and max(angles) < 20:
            speedlimit = 85
        if max(angles) >= 15 and max(angles) < 20:
            speedlimit = 90
        if max(angles) >= 10 and max(angles) < 15:
            speedlimit = 95
        if max(angles) >= 5 and max(angles) < 10:
            speedlimit = 100
        if sensing_info.speed > speedlimit:
            throttle = 0.2
        return (throttle,speedlimit)


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = "MARIO"
        return player_name


if __name__ == '__main__':
    client = DrivingClient()
    client.run()
