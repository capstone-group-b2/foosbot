#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt16
from geometry_msgs.msg import Point

ballPose = Point(0.0,0.0,0.0) 

last_ball_y_mm = 0.0

GOALIE_XPOS = 920
# Y_OFFSET = 120
# STEPS_TO_MM = (550/480)
Y_MAX_PIXELS = 600.0
Y_MIN_PIXELS = 80.0
Y_MAX_MM = 475.0
Y_MIN_MM = 0.0
MAX_STEPS = 550.0
MAX_PLAYER_TRAVEL = 130.0
MOVE_BUF = 8.0




kick_pub = None
xpos_pub = None

def pixels_to_mm(pixels):
    return (pixels - Y_MIN_PIXELS) * (Y_MAX_MM / (Y_MAX_PIXELS - Y_MIN_PIXELS))

def mm_to_steps(mm):
    return (mm * MAX_STEPS) / MAX_PLAYER_TRAVEL


def callback(point):
    global ballPose
    ballPose = point

    
    

def foosbotSpin():
    global last_ball_y_mm

    while not rospy.is_shutdown():

        # if (ballPose.x < GOALIE_XPOS > 5):
        #     kick_pub.publish()
        #     rospy.loginfo("Kicked!")

        ball_pose_y_mm = pixels_to_mm(ballPose.y)
        rospy.loginfo("X:%.2f   Y:%.2f" % (ballPose.x, ball_pose_y_mm))
        # rospy.loginfo("ball_pose_y_mm %.2f" %ball_pose_y_mm)
        # rospy.loginfo("last_ball %.2f" %last_ball_y_mm)
        if ball_pose_y_mm < 150:
            player_offset = 0
            rospy.loginfo("Player 1")
        elif ball_pose_y_mm < 300:
            rospy.loginfo("Player 2")
            player_offset = 150
        else:
            rospy.loginfo("Player 3")
            player_offset = 300 

        if (abs(last_ball_y_mm - ball_pose_y_mm) > MOVE_BUF):
            x_target = abs(mm_to_steps(ball_pose_y_mm-player_offset))
            xpos_pub.publish(int(min(x_target, MAX_STEPS)))
            last_ball_y_mm = ball_pose_y_mm

    
    
            

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
        




