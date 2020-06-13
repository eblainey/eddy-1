import rospy
from geometry_msgs.msg import Twist
from math import radians


class Lawnmower(object):

    def __init__(self, rate=rospy.Rate(10)):

        rospy.init_node('lawnmower', anonymous=False)

        # publish cmd_vel
        self.cmd_vel = rospy.Publisher('eddy/cmd_vel', Twist, queue_size=10)

        # fwd long
        self.fwd = Twist()
        self.fwd.linear.x = 0.2

        # stop and turn 90 degrees
        self.turn90 = Twist()
        self.turn90.linear.x = 0
        self.turn90.angular.z = radians(90)

        # change lane
        self.lane = Twist()
        self.lane.linear.x = 0.1
        self.lane.angular.z = 0

        # stop and turn the other way
        self.turn270 = Twist()
        self.turn270.linear.x = 0
        self.turn270.angular.z = radians(-90)

        count = 0
        while not rospy.is_shutdown():
            # 20 = 2m
            rospy.loginfo("fwd north")
            for x in range(0, 20):
                self.cmd_vel.publish(self.fwd)
                rate.sleep()
            rospy.loginfo("turn west")
            for x in range(0, 10):
                self.cmd_vel.publish(self.turn90)
                rate.sleep()
            rospy.loginfo("move west")
            # 2 = 0.2m
            for x in range(0, 2):
                self.cmd_vel.publish(self.lane)
                rate.sleep()
            rospy.loginfo("turn south")
            for x in range(0, 10):
                self.cmd_vel.publish(self.turn90)
                rate.sleep()
            rospy.loginfo("fwd south")
            for x in range(0, 20):
                self.cmd_vel.publish(self.fwd)
                rate.sleep()
            rospy.loginfo("turn east")
            for x in range(0, 10):
                self.cmd_vel.publish(self.turn270)
                rate.sleep()
            rospy.loginfo("fwd east")
            for x in range(0, 2):
                self.cmd_vel.publish(self.lane)
                rate.sleep()
            rospy.loginfo("turn north")
            for x in range(0, 10):
                self.cmd_vel.publish(self.turn270)
                rate.sleep()
            count = count + 1
            rospy.loginfo(count)
            if count == 1:
                rospy.loginfo("2 paths completed")
            elif count == 0:
                rospy.loginfo("eddy is not on track")

    def shutdown(self):
        self.cmd_vel.publish(Twist())

    rospy.sleep(1)


if __name__ == '__main__':
    try:
        Lawnmower
    except rospy.ROSInterruptException:
        pass
