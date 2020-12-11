#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import collections

rospy.init_node('SELF')

#These mappings will hold the weights for their respective velocities
linear_states_prob = {
     0.1 : .5,
     0.25 : .5,
     0.4 : .5
}
angular_states_prob =  {
    -.5: .5,
    -.45: .5,
    -.4: .5,
    -.35: .5,
    -.3: .5,
    -.25: .5,
    -.2: .5,
    -.15: .5,
    -.1: .5,
    -.05: .5,
    0 : .5,
    .05 : .5,
    .1 : .5,
    .15 : .5,
    .2: .5,
    .25: .5,
    .3: .5,
    .35: .5,
    .4: .5,
    .45: .5,
    .5: .5
}
#holds the state with the greatest weight
linear_state = 0
angular_state = 0


#variables for if we lose track of the line
counter = 0
failsafe = False

#sorts the states by weight and saves the greatest weight from each set
def obtain_state():
    global angular_states_prob
    global linear_states_prob
    global angular_state
    global linear_state
    linear_states_prob = sorted(linear_states_prob.items(), key=lambda x: x[1], reverse=True)
    angular_states_prob = sorted(angular_states_prob.items(), key=lambda x: x[1], reverse=True)
    linear_states_prob = collections.OrderedDict(linear_states_prob)
    angular_states_prob = collections.OrderedDict(angular_states_prob)
    linkeys = linear_states_prob.keys()
    angkeys = angular_states_prob.keys()
    linear_state = linkeys[0]
    angular_state = angkeys[0]


speed = None
angle = None

def get_angular_reward():
    global speed
    global angle
    angleSum = 0
    linSum = 0
    global counter
    global failsafe
    
    if cy != None:
        #check if we have lost track of the line
        #enter failsafe mode if we lose it for 10 iterations
        if cy > 1000:
            counter = counter +1
            if counter > 10:
                failsafe = True
                print 'failsafe mode active'
        else:
            counter = 0
            failsafe = False

        if cy < 800 or cy > 1000:   #cy is way off, emphasize min speed
            speed = 0.1
        elif cy > 905 or cy < 895:  #cy is slightly off, emphasize cautious speed
            speed = 0.25
        else:                       #cy is fine, emphasize cruising speed
            speed = 0.4
    else:
        speed = -1
    
    #convert from cx range to speed range: (0,1570) => (0.5, -0.5)
    if cx != None:
        angle = (float(cx)/1570.0 - 0.5) * -1.0
    else:
        angle = -1


    if speed > -1:
        #adjust weights based on closeness to current desired speed
        for value in linear_states_prob.keys():
            linSum += linear_states_prob[value]
            if abs(value - speed) < 0.05:
                linear_states_prob[value] = linear_states_prob[value]*5
            elif abs(value - speed) < 0.1:
                linear_states_prob[value] = linear_states_prob[value]*2

    #scale weights if they start getting too big
    if linSum > 5:
        for value in linear_states_prob.keys():
            linear_states_prob[value] = linear_states_prob[value]/5
            if linear_states_prob[value] < 0.05:    #puts a lower bound on less likely states
                linear_states_prob[value] = 0.05

    if angle > -1:
        #adjust weights based on closeness to current desired angular
        for value in angular_states_prob.keys():
            angleSum += angular_states_prob[value]
            if abs(value - angle) < 0.05:
                angular_states_prob[value] = angular_states_prob[value]*5
            elif abs(value - angle) < 0.1:
                angular_states_prob[value] = angular_states_prob[value]*2

    #make sure probabilities don't go off to infinity or 0
    if angleSum > 5:
        for value in angular_states_prob.keys():
            angular_states_prob[value] = angular_states_prob[value]/5
            if angular_states_prob[value] < 0.05:
                angular_states_prob[value] = 0.05


cx = None
cy = None
#hsv range for yellow so we can isolate the line
lower_yellow = numpy.array([70, 50, 75])
upper_yellow = numpy.array([100, 255, 255])

#get the camera data and extract info about the line to follow
def cv_callback(msg):
    global image_in_hsv
    global cx
    global cy

    if True:
        #get the image and convert it to hsv
        cv_image = bridge.imgmsg_to_cv2(msg)
        cv_image_2 = cv_image
        image_in_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #create a mask that makes everything outside of the yellow line black
        lower = lower_yellow
        upper = upper_yellow
        mask_colors = cv2.inRange(image_in_hsv,  lower, upper)
        mask_yellow = cv2.inRange(image_in_hsv, lower_yellow, upper_yellow)

        #use mask to identify average position of yellow pixels
        h, w, d = cv_image.shape
        moments = cv2.moments(mask_yellow)
        if moments['m00'] > 0:
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            
       
        #take the average coordinates and update states
        get_angular_reward()
        obtain_state()
        masked = cv2.bitwise_and(image_in_hsv, image_in_hsv, mask=mask_yellow)
        unmasked = cv2.cvtColor(masked, cv2.COLOR_HSV2RGB)
        real_img = bridge.cv2_to_imgmsg(unmasked)
        img_publisher.publish(real_img)
        t = Twist()

        #apply most favorable state to the movement of the robot
        if failsafe:
            if angular_state >= 0:
                t.angular.z = 0.5
            elif angular_state <= 0:
                t.angular.z = -0.5
            t.linear.x = 0.0
        else:
            t.linear.x = linear_state
            t.angular.z = angular_state

        #print('Linear States: ', linear_states_prob)
        #print ' '
        #print('Angular States: ', angular_states_prob)
        print('Linear State: ', t.linear.x)
        print('Angular State: ', t.angular.z)
        print ' '
        vel_pub.publish(t)
    

img_publisher = rospy.Publisher('/my_image', Image, queue_size=1)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cv_sub = rospy.Subscriber('camera/rgb/image_raw', Image, cv_callback)
bridge = cv_bridge.CvBridge()
rate = rospy.Rate(10.0)


while not rospy.is_shutdown():
    rospy.spin()