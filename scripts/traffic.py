import rospy
from class_meeting_08_kinematics.msg import Traffic
import numpy as np

class TrafficNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.traffic_status_pub = rospy.Publisher("/traffic_status", Traffic)

        # Counter to loop publishing direction with
        self.direction_counter = 0
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        while (not rospy.is_shutdown()):
            trafficMsg = Traffic()
            trafficMsg.direction = self.direction_counter % 3
            self.direction_counter += 1
            self.traffic_status_pub.publish(trafficMsg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('traffic_controller')
    traffic_node = TrafficNode()
    traffic_node.run()