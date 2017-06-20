'''In this exercise you need to implement an angle interpolation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interpolation or Bezier interpolation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interpolation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

        self.startTime = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {} #Final function for Bezier interpolation

        emptyTemp = ([], [], [])
        if (self.keyframes == emptyTemp):

            return target_joints

        # Starting the computation
        if self.startTime == -1:
            self.startTime = perception.time

        # Next, computes timeDiff
        timeDiff = perception.time - self.startTime

        (names, times, keys) = keyframes

        # Iterates over all joints in the keyframes
        skippedJoints = 0
        for (i, name) in enumerate(names):
            t0 = 0  # keyframe's lower threshold
            t1 = 0  # keyframe's upper threshold
            index = 0  # keyframe's upper index, to be used later for interpolation

            jointTimes = times[i]
            numJointTimes = len(jointTimes)

            # interpolation is finished for this joint, dont do any more steps
            if timeDiff > jointTimes[-1]:
                skippedJoints += 1
                # reset timer and keyframes
                if skippedJoints == len(names):
                    self.startTime = -1
                    self.keyframes = ([], [], [])
                continue

            # iterate over all times of the current joint to find the right time
            for j in xrange(numJointTimes):
                t1 = jointTimes[j]

                # we found the right interval -> break
                if ((timeDiff >= t0 and timeDiff <= t1)):
                    index = j
                    break
                t0 = t1

            # calculate t-value
            t = (timeDiff - t0) / (t1 - t0)

            # set p-values
            # Bezier interpolation is divided into two equation with different parameters.
            # It's depends on the part of the interpolation function where we are. At the beginning index =0

            # if index == 0  -> no values for p0 and p1

            if index == 0:
                p0 = 0
                p1 = 0
                p3 = keys[i][index][0]
                p2 = p3 + keys[i][index][1][2]
            else:
                p0 = keys[i][index - 1][0]
                p3 = keys[i][index][0]
                p1 = p0 + keys[i][index - 1][2][2]
                p2 = p3 + keys[i][index][1][2]

            # calculate joint angle
            angle = ((1 - t) ** 3) * p0 + 3 * t * ((1 - t) ** 2) * p1 + 3 * (t ** 2) * (1 - t) * p2 + (t ** 3) * p3

            target_joints[name] = angle

            # without this doesn't work.
            # IT is said in : http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
            # "LHipYawPitch and RHipYawPitch share the same motor so they move simultaneously and symmetrically.
            # In case of conflicting orders, LHipYawPitch always takes the priority.

            # Because of that, we have to add also the angles in RHipYawPitch when LHipYawPitch is called
            if (name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = angle
                # print degrees(angle)

                    # time_diff = perception.time - self.time_diff #Rename this variable
        # time_diff = {} #2D array with differences between target position and sensor times, for each joint
        # delta = {} #2D array with joint position differences between target and sensor
        # names, times, keys = keyframes #three separate arrays to store returned values from keyframes

        #Calculate the time differences and joint position differences between the target and sensor
        # for i in range(len(names)):
        #     timeTemp = []
        #     # deltaTemp = []
        #     for j in range(len(times[i])):
        #         timeTemp[j] = times[i][j] - perception.time #Target time (see imported) - the simulation time from sensor data
        #         if (times[i][j] < perception.time & j < times[i] - 1):
        #             if(times[i][j + 1] > perception.time):
        #                 lower = times[i][j]
        #                 higher = times[i][j + 1]
        #     # for k in range(len(keys)):
        #     #     deltaTemp[k] = keys[i][k][0] - perception.joint[names[i]]
        #     time_diff.append(timeTemp)
            # delta.append(deltaTemp)

        # delta = keys perception.joint

        # Loop through all robot joints
        # for i in range(len(names)):
        #     joint = names[i]
        #     # Update joints included in this position
        #     if joint in self.joint_names:
        #         # Look at times associated with each joint
        #         for j in range(len(times[i]) - 1):
        #             # Calculate first angle before interpolation
        #             if time_diff < times[i][0]:
        #                 target_joints[joint] = self.calculate_first_angle(times, keys, i, joint, time_diff)
        #             # Calculate between interpolation
        #             elif times[i][j] < time_diff < times[i][j + 1] and j + 1 < len(times[i]):
        #                 target_joints[joint] = self.calculate_bezier_angle(times, keys, i, j, joint, time_diff)

        return target_joints

        # def calculate_first_angle(times, keys, index, joint, time_diff):
        #     '''
        #     @param ...
        #     @param index: index of joint
        #     @return bezier(t)
        #     '''
        #     # Set time values
        #     t0 = 0.0
        #     t3 = times[index][0]
        #
        #     # Set angle values
        #     a0 = self.perception.joint[joint]
        #     a3 = keys[index][0][0]
        #
        #     # Control angles
        #     a1 = keys[index][0][1][2] + a0
        #     a2 = keys[index][0][2][2] + a3
        #
        #     dt = (time_diff) / t3
        #     return self.calculate_bezier_interpolation(a0, a1, a2, a3, dt)
        #
        # def calculate_bezier_angle(times, keys, index, t_index, joint, time_diff):
        #     '''
        #     @param ...
        #     @param index: index of joint
        #     @param t_index: time index
        #     @return bezier(t)
        #     '''
        #     # Set time values
        #     t0 = times[index][t_index]
        #     t3 = times[index][t_index + 1]
        #     # Control times
        #     t1 = keys[index][t_index][1][1] + t0
        #     t2 = keys[index][t_index][2][1] + t3
        #
        #     # Set angle values
        #     a0 = keys[index][t_index][0]
        #     a3 = keys[index][t_index + 1][0]
        #     # Control angles
        #     a1 = keys[index][t_index][1][2] + a0
        #     a2 = keys[index][t_index][2][2] + a3
        #
        #     dt = (time_diff - t0) / (t3 - t0)
        #
        #     return self.bezier_interpolation(a0, a1, a2, a3, dt)

        # @staticmethod
        # def bezier_interpolation(a0, a1, a2, a3, dt):
        #     c0 = (1 - dt) ** 3
        #     c1 = 3 * (1 - dt) ** 2
        #     c2 = 3 * (1 - dt)
        #     return c0 * a0 + c1 * a1 * dt + c2 * a2 * dt ** 2 + a3 * dt ** 3
        # Compute conditions
        # def inter_one(P, t):
        #     # find y0, y1
        #     y0 = 2 * a2;
        #     dery0 = 6 * a1 * t
        #     yt = a1 + 2 * a2 + 3 * a3 * pow(t, 2)
        #     deryt = 2 * a2 + 6 * a3 * t
        # # for n in range(len(keys)):
        # #     target_joints(n) = inter_one(P, t)
        # # Build the polynomial
        # def compute_a(t):
        #     y = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3)
        #     yPrime = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2)
        #     yDoubleprime = 2 * a2 + 6 * a3 * t
        # # Build the polynomial
        #
        #
        # # Set parameters needed for Splines interpolation
        # a0 = perception.joint
        # # a1 = delta/time_diff #TODO: calculate and avoid error
        # a2 = 0
        # a3 = 0
        # #Calculate splines interpolation
        # target_temp = []
        # for i in range(len(delta)):
        #     for j in range(len(time_diff[0])):
        #         target_temp = a0[i][j] + a1[i][j] + a2[i][j] * time_diff[i][j] * pow(time_diff[i][j], 2) + a3 * time_diff[i][j] * pow(time_diff[i][j], 3)
        #     target_joints.append(target_temp)
        # # t =  - keyframes.times[self.sensor.joints]
        # # target_joints = hello()
        # # t = keys[0][0][0] for the first splines value
        # return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    # agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    agent.run()
