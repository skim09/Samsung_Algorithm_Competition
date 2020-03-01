import setup_path
from dqn_model import DQNClient
from dqn_model import DQNParam
import math
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential
import sys

# =========================================================== #
# Training finish conditions (hour)
# assign training duration by hour : 0(limit less), 1 (an hour), 1.5 (an hour and half) ...
# =========================================================== #
training_duration = 0

# =========================================================== #
# model/weight load option
# =========================================================== #
model_load = False
model_weight_path = "./save_model/T0802_171145/dqn_weight_150.h5"


# ===========================================================

class DQNCustomClient(DQNClient):
    def __init__(self):
        self.dqn_param = self.make_dqn_param()
        super().__init__(self.dqn_param)

    # =========================================================== #
    # Tuning area (Hyper-parameters for model training)
    # =========================================================== #
    @staticmethod
    def make_dqn_param():
        dqn_param = DQNParam()
        dqn_param.discount_factor = 0.99
        dqn_param.learning_rate = 0.00025
        dqn_param.epsilon = 1.0
        dqn_param.epsilon_decay = 0.999
        dqn_param.epsilon_min = 0.01
        dqn_param.batch_size = 100
        dqn_param.train_start = 1000
        dqn_param.memory_size = 20000
        return dqn_param

    # =========================================================== #
    # Action Space (Control Panel)
    # =========================================================== #
    def action_space(self):
        # =========================================================== #
        # Area for writing code
        # =========================================================== #
        # Editing area starts from here
        #
        actions = [
            dict(throttle = 0.8, steering = 0),
            dict(throttle = 0.8, steering = 0.1),
            dict(throttle = 0.8, steering = -0.1),
            dict(throttle = 0.8, steering = 0.2),
            dict(throttle = 0.8, steering = -0.2),
            dict(throttle = 0.8, steering = 0.3),
            dict(throttle = 0.8, steering = -0.3),
            dict(throttle = 0.5, steering = 0.5),
            dict(throttle = 0.5, steering = -0.5)
        ]
        #
        # Editing area ends
        # ==========================================================#
        return actions


    #member variables
    checkpointR = 10
    nextcheckpoint = 5
    # =========================================================== #
    # Reward Function
    # =========================================================== #


    def compute_reward(self, sensing_info):
        finalreward = 0
        thresh_dist = self.half_road_limit  # 4 wheels off the track
        dist = abs(sensing_info.to_middle)
        if dist > thresh_dist:
            finalreward = -1
        elif sensing_info.collided:
            finalreward = -1
        else:
            ## Meat ##
            toMiddle = sensing_info.to_middle
            # Calculate independent angle of road ahead
            forwardAngles = [sensing_info.track_forward_angles[0]]
            for i in range(1, 10):
                forwardAngles.append(sensing_info.track_forward_angles[i]-sensing_info.track_forward_angles[i-1])
            # Calculate center based on angles ahead
            curveCenter = self.curve_center(sensing_info, forwardAngles)[0]
            finalCurveCenter = self.curve_center(sensing_info, forwardAngles)[1]
            # Add obstacles to list
            obstacleList = []
            for obstacle in sensing_info.track_forward_obstacles:
                if obstacle['dist'] >= 0:
                    if obstacle['to_middle'] <= (self.half_road_limit+1):
                        obstacleList.append(obstacle)
            # if there is an obstacle(s) ahead calculate the center acoordingly
            if len(obstacleList) > 0:
                obstacleCenter = self.obstacle_center(sensing_info, obstacleList, curveCenter, finalCurveCenter)
                calculatedCenter = self.calculate_obstacle_center(sensing_info, curveCenter, obstacleCenter, obstacleList)
            else:
                calculatedCenter = curveCenter

            # how far from center?
            toMiddle -= calculatedCenter
            # what "zone" are we in?
            zone = self.calculate_zone(toMiddle)
            # predictive angle so car starts turning beforehand
            angle = sensing_info.moving_angle - (forwardAngles[0]+forwardAngles[1])/2
            # calculate a basic speed limit
            speedlimit = self.speedlimit(sensing_info, zone, forwardAngles, angle)

            finalreward = self.reward_calculator(sensing_info, toMiddle, calculatedCenter, thresh_dist, forwardAngles, angle, speedlimit, zone)

            ## attempt at making a time-dependent reward, but does not work well = TO FIX
            self.checkpointR -= 0.01
            if sensing_info.lap_progress >= self.nextcheckpoint:
                self.nextcheckpoint += 5
                finalreward += self.checkpointR
                self.checkpointR = 10
        #
        # Editing area ends
        # ==========================================================#
        return finalreward



    ################################# CUSTOM FUNCTIONS ##################################
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
            closestCenter = min(goodCenters, key=lambda x:abs(x-sensing_info.to_middle))
            obstacleCenter = closestCenter
        return obstacleCenter

    def calculate_zone(self, toMiddle):
        zone = 0
        if toMiddle < -(self.half_road_limit*3/4): zone = 1
        if toMiddle < -(self.half_road_limit/2): zone = 2
        if toMiddle < -(self.half_road_limit/4): zone = 3
        if toMiddle <= (self.half_road_limit/4): zone = 4
        if toMiddle <= (self.half_road_limit/2): zone = 5
        if toMiddle <= (self.half_road_limit*3/4): zone = 6
        if toMiddle > (self.half_road_limit*3/4): zone = 7
        return zone

    def speedlimit(self, sensing_info, zone, forwardAngles, angle):
        speedlimit = 130
        angles = list(map(abs, forwardAngles[4:9]))
        if max(angles) >= 25:
            speedlimit = 80
        if max(angles) >= 20 and max(angles) < 20:
            speedlimit = 85
        if max(angles) >= 15 and max(angles) < 20:
            speedlimit = 90
        if max(angles) >= 10 and max(angles) < 15:
            speedlimit = 95
        return speedlimit


    def reward_calculator(self, sensing_info, toNewMiddle, center, thresh_dist, forwardAngles, angle, speedlimit, zone):
        # REWARDS
        base = 10

        distR = 0
        if zone == 1 or zone == 7:
            distR = 0.2
        if zone == 2 or zone == 6:
            distR = 0.4
        if zone == 3 or zone == 5:
            distR = 0.6
        if zone == 4:
            distR = 1.0

        speedR = 0
        speedratio = sensing_info.speed/speedlimit
        if sensing_info.speed >= 0:
            speedR = 0.0
        elif sensing_info.speed >= 20:
            speedR = 0.1
        elif sensing_info.speed >= 30:
            speedR = 0.2
        elif sensing_info.speed >= 40:
            speedR = 0.3
        elif sensing_info.speed >= speedlimit - 35:
            speedR = 0.4
        elif sensing_info.speed >= speedlimit - 25:
            speedR = 0.6
        elif sensing_info.speed >= speedlimit - 15:
            speedR = 0.8
        elif sensing_info.speed >= speedlimit - 5:
            speedR = 1.0
        elif sensing_info.speed >= speedlimit + 5:
            speedR = 0.8
        elif sensing_info.speed >= speedlimit + 15:
            speedR = 0.6
        elif sensing_info.speed >= speedlimit + 25:
            speedR = 0.4
        elif sensing_info.speed >= speedlimit + 35:
            speedR = 0.2

        angleR = 1.0
        for i in range(1, 6):
            if abs(sensing_info.moving_angle) > i*3:
                angleR -= 0.2

        # proportional rewards
        return (distR*0.3 + speedR*0.4 + angleR*0.3)

    ############################ CUSTOM FUNCTION END ##################################

    # =========================================================== #
    # Model network
    # =========================================================== #
    def build_custom_model(self):
        model = Sequential()
        model.add(Dense(32, input_dim=self.state_size, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(32, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(self.action_size, activation='linear',
                        kernel_initializer='he_uniform'))
        model.summary()

        model.compile(loss='mse', optimizer=Adam(lr=self.dqn_param.learning_rate))

        return model


if __name__ == "__main__":
    client = DQNCustomClient()

    client.override_model()

    if model_load:
        client.agent.load_model(model_weight_path)

    client.run(training_duration)
    sys.exit()
