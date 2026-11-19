#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_msgs.msg import Bool

class InspectionChecker:
    def __init__(self, namespace):
        self.namespace = namespace
        self.trajectory_points = []
        self.current_point_index = 0
        self.current_odometry = None
        self.done = False

        # Publisher
        self.done_pub = rospy.Publisher(namespace + "/done_inspection", Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber(namespace + "/inspection_trajectory", MultiDOFJointTrajectory, self.trajectory_callback)
        rospy.Subscriber(namespace + "/odometry", Odometry, self.odometry_callback)

    def trajectory_callback(self, msg):
        if msg.points:
            self.trajectory_points = msg.points
            self.current_point_index = 0
            self.done = False
            rospy.loginfo(f"Received new trajectory for {self.namespace}. Number of points: {len(self.trajectory_points)}")
            rospy.loginfo("I'm in the function process.")
            self.process_trajectory()

    def odometry_callback(self, msg):
        self.current_odometry = msg.pose.pose

    def process_trajectory(self):
        rospy.loginfo(f"Processing trajectory for {self.namespace}.")
        rospy.loginfo(f"Current point index: {self.current_point_index}")
        rospy.loginfo(f"Total points in trajectory: {len(self.trajectory_points)}")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown() and not self.done:
            if self.current_odometry:
                if self.current_point_index < len(self.trajectory_points):
                    current_pose = self.current_odometry
                    target_pose = self.trajectory_points[self.current_point_index].transforms[0]

                    rospy.loginfo(f"Current pose: {current_pose}")
                    rospy.loginfo(f"Target pose: {target_pose}")

                    if self.is_pose_equal(current_pose, target_pose):
                        rospy.loginfo(f"Odometry matches point {self.current_point_index + 1} of the trajectory for {self.namespace}.")
                        print(f"{self.namespace} reached point {self.current_point_index + 1} of the trajectory.")
                        self.current_point_index += 1

                        if self.current_point_index == len(self.trajectory_points):
                            self.done_pub.publish(Bool(data=True))
                            rospy.loginfo(f"{self.namespace} reached the last pose of the trajectory.")
                            print(f"{self.namespace} reached the last pose of the trajectory.")
                            self.done = True
                else:
                    rospy.loginfo(f"All points in the trajectory have been processed for {self.namespace}.")
                    self.done = True
            else:
                rospy.loginfo(f"No odometry data available for {self.namespace}.")

            rate.sleep()

    @staticmethod
    def is_pose_equal(odom_pose, traj_pose, pos_tolerance=1, ori_tolerance=1):
        # Check if the position and orientation are within the given tolerance
        pos_equal = (abs(odom_pose.position.x - traj_pose.translation.x) < pos_tolerance and
                     abs(odom_pose.position.y - traj_pose.translation.y) < pos_tolerance and
                     abs(odom_pose.position.z - traj_pose.translation.z) < pos_tolerance)

        ori_equal = (abs(odom_pose.orientation.x - traj_pose.rotation.x) < ori_tolerance and
                     abs(odom_pose.orientation.y - traj_pose.rotation.y) < ori_tolerance and
                     abs(odom_pose.orientation.z - traj_pose.rotation.z) < ori_tolerance and
                     abs(odom_pose.orientation.w - traj_pose.rotation.w) < ori_tolerance)

        return pos_equal and ori_equal

if __name__ == '__main__':
    try:
        rospy.init_node('inspection_checker')

        # Create instances for /red and /blue namespaces
        red_checker = InspectionChecker("/red")
        blue_checker = InspectionChecker("/blue")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            red_checker.process_trajectory()
            blue_checker.process_trajectory()
            rate.sleep()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
