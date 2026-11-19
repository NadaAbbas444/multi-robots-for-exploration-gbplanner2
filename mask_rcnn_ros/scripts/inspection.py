#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, Pose
from mask_rcnn_ros.msg import CenterAndRadius
from visualization_msgs.msg import Marker
import numpy as np
import random

class ShapePublisher:
    def __init__(self):
        rospy.init_node('shape_to_points_publisher', anonymous=True)
        self.shape_sub = rospy.Subscriber('/bounding_box_center_and_radius', CenterAndRadius, self.shape_callback)
        self.points_pub = rospy.Publisher('/points_topic', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)  # Define the marker publisher

        self.num_sides = 8 # Example: Hexagon
        self.height_range = (2.0, 3.0)  # Minimum and maximum heights
        self.last_polygon = []  # Store vertices of the last polygon

    def shape_callback(self, msg):
        radius = 5* msg.radius
        current_center = (msg.center.x, msg.center.y)
        # center = (
        #     current_center[0] + radius*np.cos(current_center[1] / current_center[0]),
        #     current_center[1] + radius*np.sin(current_center[1] / current_center[0])
        # )   

        # Ensure the last_polygon is not empty before performing the check
        if self.last_polygon and self.is_point_in_polygon(current_center, self.last_polygon):
            # rospy.loginfo("Ignoring center inside the last polygon.")
            return
        # print("current_center",current_center,"self.last_polygon",self.last_polygon)
        print("current_center",current_center,"radius",radius)

        # Correct calculation assuming current_center is a numpy array 
        base_vertices, top_vertices = self.create_random_shape(current_center, radius, self.num_sides, self.height_range)
        self.last_polygon = base_vertices  # Update last polygon
        self.publish_points(top_vertices)
        self.publish_markers(top_vertices)


    def is_point_in_polygon(self, point, polygon):
        """ Check if point (x, y) is inside the polygon defined by list of vertices [(x1, y1), (x2, y2), ...] """
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def create_random_shape(self, base_center, base_radius, num_sides, height_range):
        base_vertices = self.calculate_polygon_vertices(base_center[0], base_center[1], base_radius, num_sides)
        top_vertices = [(x, y, random.uniform(*height_range)) for x, y in base_vertices]
        return base_vertices, top_vertices
    
    def calculate_polygon_vertices(self, cx, cy, radius, num_sides, orientation=0):
        angle_between_vertices = 2 * np.pi / num_sides
        vertices = []
        for i in range(num_sides):
            theta = orientation + i * angle_between_vertices
            x = cx + radius * np.cos(theta)
            y = cy + radius * np.sin(theta)
            vertices.append((x, y))
        return vertices

    def publish_points(self, top_vertices):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "world"
        for vertex in top_vertices:
            pose = Pose()
            pose.position.x = vertex[0]
            pose.position.y = vertex[1]
            pose.position.z = vertex[2]
            pose_array.poses.append(pose)
        self.points_pub.publish(pose_array)

    def publish_markers(self, points):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "shapes"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # Sphere diameter
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        for p in points:
            point = Pose()
            point.position.x = p[0]
            point.position.y = p[1]
            point.position.z = p[2]
            marker.points.append(point.position)
        self.marker_pub.publish(marker)
if __name__ == '__main__':
    try:
        sp = ShapePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
