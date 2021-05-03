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
            #print('current pos: ' + str(current_pos))
            current_angle = current_pos[2]

            #calculate the goal angle
            relative_x = goal_pos[0]-current_pos[0]
            relative_y = goal_pos[1]-current_pos[1]
            goal_angle = math.atan2(relative_y, relative_x)
            #print('goal angle: ' + str(goal_angle))
            #break if within .1 m
            if (posDiff(current_pos, goal_pos) < .5 ):
                break

            #calculate angle speed and lin speed drive
            ang_error = angleDiff(current_angle, goal_angle)
            pos_error = posDiff(current_pos, goal_pos)
            #print('error: ' + str(ang_error) + ' ' +str(pos_error))

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
            #print('speed: ' + str(ang_speed) + ' ' + str(lin_speed))

            #set old values
            old_ang_error=ang_error
            old_pos_error=pos_error
            rate.sleep()
            #print(' ')

        self.r.drive(angSpeed=0, linSpeed=0)

    def start(self):
        return self.r.getMCLPose()
        '''
        try:
            return self.r.getMCLPose()
        except:
            return self.r.getPositionTup()
        '''

    def pickup(self,color):
        if color == 'GREEN':
            light = (40,15,20)
            dark = (80,255,235)

        if color == 'PURPLE':
            light = (140,15,20)
            dark = (165,255,235)

        if color == 'RED':
            light = (0,20,50)
            dark = (8,255,235)

        if color == 'YELLOW':
            light = (25,15,20)
            dark = (40,255,235)

        if color == 'BLUE':
            light = (105,0,0)
            dark = (115,255,255)

        ros = rospy.Rate(30)
        count = 0

        while not rospy.is_shutdown():
            img = self.r.getImage()
            dpth = self.r.getDepth()
            mask = getMask(light, dark, img)
            augmented = augmentedImg(img, mask, color)
            bDist = getBalloonDist(mask, dpth)
            err = 320-findCoM(light,dark,img, mask)
            if err > .15:
                err = .15
            if err < -.15:
                err = -.15
            if err==0:
                self.r.drive(angSpeed=3,linSpeed=0)
            else:
                self.r.drive(angSpeed=err,linSpeed=.1)
            if bDist < 750 and bDist >0:
                count += 1
                if (count>1):
                    break   
            else:
                count = 0 
            ros.sleep()
        self.r.drive(angSpeed=0, linSpeed=0)





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

def get_ang_err(com, P=.5):
    return P*(320-com)

def get_dist_err(dens,P=.5):
    return P*(921600-dens)

def augmentedImg(image, mask, color):
    if color == 'BLUE' or color == 'PINK':
        # apply blue filter
        image[:,:,0] = np.bitwise_or(image[:,:,0], mask)
    if color == 'YELLOW' or color == 'GREEN':
        # apply green filter
        image[:,:,1] = np.bitwise_or(image[:,:,1], mask)
    if color == 'PINK' or color == 'RED' or color == 'YELLOW':
        # apply red filter
        image[:,:,2] = np.bitwise_or(image[:,:,2], mask)

    return image

def getMask(light, dark, img):
    hsv_test = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_test, light, dark)
        mask = cv2.fastNlMeansDenoising(mask, mask, 100, 7, 21)
        ret, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow('mask.jpg',mask)
        cv2.waitKey(1)
    return mask

def findCoM(light, dark, img, mask):

    #mask = getMask(light, dark, img)

    array = np.zeros(len(mask[0]))
    num = 0

    for col in range(len(mask[0])):
        for row in range(len(mask)):
            array[col] += mask[row,col]
            num += mask[row,col]

    sum = 0
    i = 0
    for x in array:
        sum += x*i
        i +=1


    if (num!=0):
        x = sum/num

    if not (math.isnan(x)):
        for row in range(len(mask)):
            mask[row,int(x)] = 0

    #print(x)

    #cv2.imwrite("mask.jpg", mask)

    return x

def getBalloonDist(mask, depth):
    totDist = 0
    numPixels = 0
    xx = 0
    yy = 0
    for x in mask:
        for y in x:
            if y == 255:
                totDist += depth[xx, yy]
                numPixels += 1
            yy += 1

            yy = 0
            xx += 1
    if numPixels == 0:
        return 10000
    return totDist/numPixels    