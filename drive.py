import math, rospy
from turtleAPI import robot

class Driver:
    def __init__(self):
        # make the robot and lists for tracking error
        self.r = robot()
        self.error_list_pos = []
        self.error_list_angle = []
        
    def goto(self,x,y):
        goal_pos = (x, y)

        # loop until at position
        old_ang_error = 0
        old_pos_error = 0
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            speed_limit = 4
            turn_limit = .4

            #current pos
            current_pos = self.r.getMCLPose()
            '''
            try:
                current_pos = self.r.getMCLPose()
            except:
                current_pos = self.r.getPositionTup()
            '''
            print('current pos: ' + str(current_pos))
            current_angle = current_pos[2]

            #calculate the goal angle
            relative_x = goal_pos[0]-current_pos[0]
            relative_y = goal_pos[1]-current_pos[1]
            goal_angle = math.atan2(relative_y, relative_x)
            print('goal angle: ' + str(goal_angle))
            #break if within .1 m
            if (posDiff(current_pos, goal_pos) < .5 ):
                break

            #calculate angle speed and lin speed drive
            ang_error = angleDiff(current_angle, goal_angle)
            pos_error = posDiff(current_pos, goal_pos)
            print('error: ' + str(ang_error) + ' ' +str(pos_error))

            #speed
            ang_speed = pid_speed(-.5, 0, -.01, ang_error, old_ang_error, self.error_list_angle)
            lin_speed = pid_speed(.05, 0, .01, pos_error, old_pos_error, self.error_list_pos)

            #set speed limit
            if lin_speed > speed_limit:
                lin_speed = speed_limit

            #set turn limit
            if (ang_speed > turn_limit):
                ang_speed = turn_limit
            elif (ang_speed < -turn_limit):
                ang_speed = -turn_limit

            self.r.drive(angSpeed=ang_speed, linSpeed=lin_speed)
            print('speed: ' + str(ang_speed) + ' ' + str(lin_speed))

            #set old values
            old_ang_error=ang_error
            old_pos_error=pos_error
            rate.sleep()
            print(' ')

        self.r.drive(angSpeed=0, linSpeed=0)

    def start(self):
        return self.r.getMCLPose()
        '''
        try:
            return self.r.getMCLPose()
        except:
            return self.r.getPositionTup()
        '''
# pid
def pid_speed(self,kp, ki, kd, error, old_error, error_list):
    # add the error to the integral portion
    if len(error_list) > 5:
        error_list.pop()
    error_list.append(error)

    #calculate sum
    error_sum = 0
    for i in self.error_list:
        error_sum += i

    # kp portion + ki portion
    to_return = (kp * error) + (ki * error_sum)
    to_return += kd * (error - old_error)

    return to_return

#error function for angle
def angleDiff(cur_angle, desired):
    # calculate difference
    diff = cur_angle - desired
    while diff > math.pi:
        diff -= 2*math.pi
    while diff < -math.pi:
        diff += 2*math.pi

    #if (abs(diff) < .1):
    #   if diff > 0: return .1
    #    if diff < 0: return -.1

    if (abs(diff) > 3):
        if diff > 0: return 3
        if diff < 0: return -3

    return diff

# error function for position
def posDiff(current, desired):
    #calculate component differences
    x_diff = current[0] - desired[0]
    y_diff = current[1] - desired[1]

    #calculate the total distance
    return (x_diff**2 + y_diff**2)**.5

