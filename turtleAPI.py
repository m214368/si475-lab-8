import rospy
from kobuki_msgs.msg import Led
import roslib
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
import numpy as np
import math
class robot():
  """
  robot encapsulates all of the needed nodes to make the functions interface
  with the ros hardware.

  Specific functions will only work if those nodes have been started (for
  example, .turn() will only work if the robot's "legs" have been turned on)
  """

  def __init__(self):
    """
    This constructor connects to the robot and starts the communication with
    the robot and the kinect.
    """

    """
    mapNode: Subscriber to the google cartographer map publisher, handles by __mapHandle
    cmd_vel: Publisher to robot hardware that allows robot to move
    led1:	Publisher for pushing light commands to the robots Led
    bridge: 	object that opencv uses to convert the 3d cameras image to something Opencv can use
    image_sub:	node that subscribs to camera node and sends to handeler
    state:	var for initial bumber state, if any bump is felt it will no longer be -1
    bumber:	var for initial bumber state, if bumped, will no longer be -1
    bumperStatus:	node for getting bumper action and sending it to the handeler
    """


    ###############note################
    #if you think you are not connecting to a node, use the turtle up commands and in a different terminal
    # type 'rostopic list' to see if the desired node is there, you can also put the below functions in try/except blocks
    
    #node for controlling light
    rospy.init_node('robotAPI', anonymous=False)

    #node for directionchange + movment
    self.cmd_vel = rospy.Publisher('/si475/usercmd', Twist, queue_size=10)


    # tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")
    rospy.on_shutdown(self.shutdown)


    #publisher for led1 light, part of light()
    self.led1 = rospy.Publisher('/mobile_base/commands/led1',Led, queue_size=10)

    #publisher and bridges for ros immagery to openCV images
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.__imgHandle)
    #self.depth_sub = rospy.Subscriber("/camera/depth/image_rect",Image,self.__depthHandle)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.__depthHandle)

    #node for bubmer sensors
    self.bumperStatus = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.__bumperHandle)
    #setting initial defualt values
    self.state = "-1"
    self.bumper = "-1"

    #subscriber for getting odometry
    try:
      self.odomSub = rospy.Subscriber("/odom", Odometry, self.__handleOdom)
    except Exception,e:
      print(str(e))			
    try:
      self.mclPoseSub=rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.__handleMCLPose)
    except Exception,e:
      print(str(e))			
    self.cv_image=None
    self.depth_image=None
    self.light()

  def drive(self,angSpeed=0,linSpeed=0):
    '''
    Uses the continuous_driver package to start the robot driving at the given
    speed.  Will return immediately, and the robot will keep moving at that
    speed until told otherwise.
    '''
    move_cmd = Twist()
    move_cmd.linear.x=linSpeed
    move_cmd.angular.z=angSpeed
    self.cmd_vel.publish(move_cmd)
  
  def stop(self):
    '''
    Uses drive() to stop the robot.
    '''
    self.drive(angSpeed=0,linSpeed=0)

  def __handleOdom(self,data):
    """
    __handleOdom puts the robots info in a variable for the software to use 
    :param odom:	stores the odom data in program accesable area
    """
    try:
      #get the odom data here
      self.odom = data.pose
    except Exception,e:
      rospy.loginfo(str(e))
  
  def __handleMCLPose(self,data):
    """
    __handleMCLPose puts the robots info in a variable for the software to use 
    """
    try:
      #get the odom data here
      self.mcl = data.pose.pose
    except Exception,e:
      rospy.loginfo(str(e))

  #gets the yaw pitch and roll.	
  def getAngle(self):
    """
    :return: a Tuple with roll, pitch, and yaw (presumably the first two are 0
    for this robot, unless things have gone quite wrong)
    """
    #:param quater: 	var for holding the quatderion information
    #:param euler:	var that contains yaw,pitch, and roll, attained from transforming the quat... informaiton from Pose

    try:
      quater = (self.odom.pose.orientation.x ,self.odom.pose.orientation.y,self.odom.pose.orientation.z,self.odom.pose.orientation.w)
      #magic function that does math from quaternions to yaw pitch and roll.
      euler = tf.transformations.euler_from_quaternion(quater)	
      return euler
    except Exception,e:
      print(str(e))
  
  #gets the yaw pitch and roll.	
  def getMCLAngle(self):
    """
    :return: a Tuple with roll, pitch, and yaw (presumably the first two are 0
    for this robot, unless things have gone quite wrong)
    """
    #:param quater: 	var for holding the quatderion information
    #:param euler:	var that contains yaw,pitch, and roll, attained from transforming the quat... informaiton from Pose

    try:
      quater = (self.mcl.orientation.x ,self.mcl.orientation.y,self.mcl.orientation.z,self.mcl.orientation.w)
      #magic function that does math from quaternions to yaw pitch and roll.
      euler = tf.transformations.euler_from_quaternion(quater)	
      return euler
    except Exception,e:
      print(str(e))

  def getPosition(self):
    """
    :return: robot's entire odom data
    """
    return self.odom

  def getPositionTup(self):
    """
    :return: (x,y,yaw) tuple
    """
    p=self.odom.pose.position
    yaw=self.getAngle()[2]
    return (p.x,p.y,yaw)

  def getMCLPose(self):
    """
    :return: (x,y,yaw) tuple
    """
    p=self.mcl.position
    yaw=self.getMCLAngle()[2]
    return (p.x,p.y,yaw)

  def __getPositionFine(self):
    return self.odom.pose.position

  def __bumperHandle(self,data):
    """
    __bumperHandle is called whenever the software receives a message from the robot detailing a bmper sate change

    :param state:	State of bumber [-1:no change, 0:released, 1:pressed
    :param bumper: 	Number of bumper pressed
    """
    try:
      #update the bumber state
      self.state = data.state
      self.bumper = data.bumper
    except Exception,e:
      rospy.loginfo(str(e))


  def getBumpStatus(self):
    """
    getBumpStatus returns the bumper states to the User
    """
    try:
      #returning a tuple with the response information
      return {'bumper':self.bumper,'state':self.state}
    except Exception,e:
      print(str(e))

  def getDepth(self):
    """
    Returns a depth image, where the each pixel is a single float containing the distance from the camera of that pixel.  "I don't know"s are nans.
    """
    return self.depth_image
  
  def __depthHandle(self,data):
    """
    __depthHandle handles depth images that are published from the 3d camera, sends to a 
    opencv bridge that converts the information and assings to the depth_image vars
    """
    #:param depth_image:	var for holding the resulting image from the camera

    try:
      self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    #code below will display on screen under proper conditions
    #cv2.imshow("Image window", self.depth_image)
    #cv2.waitKey(3)


  #function for handleing the image 
  def __imgHandle(self,data):
    """
    __imgHandle handles images that are published from the 3d camera, sends to a 
    opencv bridge that converts the information and assings to the cv_image vars

    :param cv_image:	var for holding the resulting image from the camera
    """
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = self.cv_image.shape
    #code below will display on screen under proper conditions
    #cv2.imshow("Image window", self.cv_image)
    #cv2.waitKey(3)

  def getImage(self):
    """
    :return: the most recent image taken by the camera

    """
    #cv2.imshow("Image window", self.cv_image)
    #cv2.waitKey(3)
    return self.cv_image

  def light(self):
    """
    Flashes the light on the robot (useful for checking connection or for raves)
    """
    #r:	var used  to control the flashing frequenzy in hz
    # cmd:	data structure for the led() information, for more information refer to msg docs on Led()
    
    #create a publisher which talks to the lights
    #rate at chich to publish
    r = rospy.Rate(10);
    #create the message to  the node
    cmd = Led()
    #set value 
    cmd.value = 1	
    t = 10
    while not rospy.is_shutdown() and t > 0:
      cmd.value = 1	
      #publish the message
      self.led1.publish(cmd)
      rospy.loginfo("published the info to the node")
      r.sleep()
      cmd.value = 3
      self.led1.publish(cmd)
      r.sleep()
      t = t-1
    rospy.sleep(1)		

  def shutdown(self):
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    self.led1.publish(Led())

if __name__ == '__main__':
  try:
    print("creating robot")
    r= robot()
    print("attempting to light")
    r.light()
    r.drive(angSpeed=.3)
    while True:
      pass
    '''
    #print("trying to get pic")
    #img = r.getImage()
    #img = r.getDepth()
    #(width,height)=img.shape

    #val = np.nanmax(img)
    while True:
      #dpth = r.getDepth()
      #dpth=dpth/np.nanmax(dpth)
      #cv2.imshow("Depth", dpth)
      #img = r.getImage()
      #cv2.imshow("Image", img)
      #cv2.waitKey(1)
      bumperblue = r.getBumpStatus()
      print bumperblue
      if bumperblue['state']==0:
        move_cmd = Twist()
        move_cmd.linear.x = .5
        r.cmd_vel.publish(move_cmd)
      else:
        turn_cmd = Twist()
        turn_cmd.linear.x = -.05
        turn_cmd.angular.z = 1; 
        r.cmd_vel.publish(turn_cmd)
        
    

    print("\n attempting to get pose")
    testdata = r.getPosition()
    print(testdata)

    print("getting the euler angle from internal odom")
    euler = r.getAngle()
    print("current position")
    print(r.getPositionTup())
    print("\nattempting turn")
    r.turn(.5)
    r.driveDistance(.25)
    print("\n printing map")
    r.printMap()
      '''

  except Exception as e:
    print(e)
    rospy.loginto("node now terminated")
