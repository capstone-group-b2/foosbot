#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt16
from geometry_msgs.msg import Point

ballPose = Point(0.0,0.0,0.0) 

last_ball_y = 0.0

GOALIE_XPOS = 920
Y_OFFSET = 120
STEPS_TO_MM = (550/480)

kick_pub = None
xpos_pub = None


def callback(point):
    global ballPose
    ballPose = point
    rospy.loginfo("X:%.2f   Y:%.2f" % (ballPose.x, ballPose.y))

    
    

def foosbotSpin():
    global last_ball_y

    while not rospy.is_shutdown():

        # if (ballPose.x < GOALIE_XPOS > 5):
        #     kick_pub.publish()
        #     rospy.loginfo("Kicked!")

        if (last_ball_y != ballPose.y):
            xpos_pub.publish(abs(int((ballPose.y - Y_OFFSET) * STEPS_TO_MM)))
            last_ball_y = ballPose.y 

    
    
            

def getBallPose():
    return rospy.wait_for_message('ball', Point)


def init():
    global kick_pub, xpos_pub

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('foosbot', anonymous=True)

    rospy.Subscriber("ball", Point, callback)
    kick_pub = rospy.Publisher("kick", Empty, queue_size=0)
    xpos_pub = rospy.Publisher("xpos", UInt16, queue_size=0)


if __name__ == '__main__':
    init()
    
    try:
        foosbotSpin()
    except rospy.ROSInterruptException:
        pass
        




