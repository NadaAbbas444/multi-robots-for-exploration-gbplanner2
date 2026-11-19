#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32  # Import for the counter publisher
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Point32
from sensor_msgs.msg import Image, CameraInfo
from mask_rcnn_ros.msg import Result
from visualization_msgs.msg import Marker, MarkerArray
from planner_semantic_msgs.msg import SemanticPoint, SemanticClass
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from planner_msgs.srv import DistanceAndRole, DistanceAndRoleRequest  # Adjust with actual service name and package

from std_srvs.srv import Trigger

class SemanticDepthProcessor:
    def __init__(self):
        rospy.init_node('semantic_depth_processor')
        self.bridge = CvBridge()
        self.self_role = rospy.get_param('~me', 'default_self_role') # self_role  is the name space
        self.other_role = rospy.get_param('~other_robot', 'default_other_role') # if self_role
        self.on_hold_points = []  # List to store points that are on hold during inspection
        self.namespace = rospy.get_namespace().rstrip('/')
        self.other_namespace = 'blue' if self.namespace.endswith('red') else 'red'
        self.my_distance_service = rospy.ServiceProxy(f"/{self.namespace}/dta_node/compute_distance_and_role", DistanceAndRole)
        self.other_distance_service = rospy.ServiceProxy(f"/{self.other_namespace}/dta_node/compute_distance_and_role", DistanceAndRole)

        self.global_planner_service_available = False
        self.done_inspection = False
        self.object_count = 0  # Counter for detected objects

        self.check_planner_service_availability()
        self.initialize_nodes()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.current_segmentation = None
        self.camera_info = None
        self.previous_semantic_points = []


    def initialize_nodes(self):
        self.toggle_planning_client = rospy.ServiceProxy(self.namespace + '/gbplanner/toggle_planning', SetBool)
        rospy.Subscriber(self.namespace + "/done_inspection", Bool, self.callback)
        self.depth_sub = rospy.Subscriber(self.namespace + "/mask_rcnn/depth_sync", Image, self.depth_callback)
        self.segmentation_sub = rospy.Subscriber(self.namespace + "/mask_rcnn/result", Result, self.segmentation_callback)
        self.camera_info_sub = rospy.Subscriber(self.namespace + "/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.objects_counter_pub = rospy.Publisher(self.namespace +"/detected_objects_counter", Int32, queue_size=10)
        self.segment_vicinity_marker = rospy.Publisher(self.namespace + "/segment_vicinity", Marker, queue_size=10)
        self.previous_semantic_pub = rospy.Publisher(f"{self.namespace}/previous_semantic_points", SemanticPoint, queue_size=10)
        rospy.Subscriber(self.other_namespace+"/previous_semantic_points", SemanticPoint, self.other_semantic_callback)

        # Dynamic publisher setup based on role
        if self.self_role == "explorer":
            # rospy.loginfo(" Hello i'm the explorer ")
            self.semantic_location_pub_to_explorer = rospy.Publisher(self.namespace + "/semantic_location", SemanticPoint, queue_size=10)
            self.semantic_location_pub_to_inspector = rospy.Publisher(self.other_namespace + "/semantic_location", SemanticPoint, queue_size=10)
        else:
            # rospy.loginfo(" Hello i'm the inspector ")
            self.semantic_location_pub_to_explorer = rospy.Publisher(self.other_namespace + "/semantic_location", SemanticPoint, queue_size=10)
            self.semantic_location_pub_to_inspector = rospy.Publisher(self.namespace + "/semantic_location", SemanticPoint, queue_size=10)
    def publish_circle_marker(self, point, radius=15.0):
        # Create a marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "semantic_points"
        marker.id = self.object_count  # Use object count as a unique ID for markers
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2 * radius  # Diameter
        marker.scale.y = 2 * radius  # Diameter
        marker.scale.z = 0.1  # Small height for the cylinder to appear as a circle
        marker.color.a = 0.2  # 50% opacity
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green color
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()  # 0 or rospy.Duration() means never to auto-delete

        # Publish the marker
        self.segment_vicinity_marker.publish(marker)

    def check_distance_service_availability(self):
        """ Check availability of distance computation services in both namespaces. """
        try:
            rospy.wait_for_service(f"/{self.namespace}/dta_node/compute_distance_and_role", timeout=10)  # Increased timeout
            rospy.wait_for_service(f"/{self.other_namespace}/dta_node/compute_distance_and_role", timeout=10)
            # rospy.loginfo("Both distance services are available.")
        except rospy.ROSException:
            # rospy.logwarn("One of the distance services is not available.")
            return False
        return True


    def check_planner_service_availability(self):
        '''        
        Makes sure not to start the node before the global planner is on
        '''
        while not rospy.is_shutdown():
            try:
                # Check the availability of the planner service
                planner_service_name = f"{self.namespace}/gbplanner/global"
                rospy.wait_for_service(planner_service_name, timeout=5)
                self.global_planner_service_available = True
                # rospy.loginfo("Planner service is available.")
                break
            except rospy.ROSException:
                self.global_planner_service_available = False
                # rospy.logwarn("Planner service is not available. Retrying in 5 seconds...")
                rospy.sleep(5)
    
    def publish_object_count(self):
            # Publish the current object count
            self.objects_counter_pub.publish(self.object_count)


    def callback(self, data):
            self.done_inspection = data.data
            if self.done_inspection:
                self.activate_planner()

    def activate_planner(self):
        try:
            # Adding a 20-second delay before activating the planner
            rospy.loginfo("Waiting 20 seconds before activating the planner.")
            rospy.sleep(20)
            
            for point, semantic_type, description in self.on_hold_points:
                    rospy.loginfo(f"before on_hold_points {self.on_hold_points})")
                    self.publish_semantic_location(point, semantic_type)
                    rospy.loginfo(f"after on_hold_points {self.on_hold_points})")
                    rospy.sleep(20)
            self.on_hold_points.clear()
            
            response = self.toggle_planning_client(True)
            if response.success:
                rospy.loginfo("Planner activated.")
                # Trigger the automatic planning service
                rospy.wait_for_service(f"{self.namespace}/planner_control_interface/std_srvs/stop")
                stop_service = rospy.ServiceProxy(f"{self.namespace}/planner_control_interface/std_srvs/stop", Trigger)
                stop_service()

                rospy.wait_for_service(f"{self.namespace}/planner_control_interface/std_srvs/automatic_planning")
                automatic_planning_service = rospy.ServiceProxy(f"{self.namespace}/planner_control_interface/std_srvs/automatic_planning", Trigger)
                automatic_planning_service()

            else:
                rospy.logwarn("Failed to activate planner.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


    def handle_object_detection(self, depth_image, i):
        class_name = self.current_segmentation.class_names[i]
        score = self.current_segmentation.scores[i]
        if class_name in ["car", "truck", "chair"] and score > 0.75:
            self.process_object(depth_image, self.current_segmentation.boxes[i], class_name)

    def depth_callback(self, msg):
        if not self.global_planner_service_available:
            rospy.logwarn("Service not available.")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            if self.current_segmentation and self.current_segmentation.boxes:
                min_length = min(len(self.current_segmentation.boxes), len(self.current_segmentation.class_names), len(self.current_segmentation.scores))
                for i in range(min_length):
                    self.handle_object_detection(depth_image, i)
        except Exception as e:
            rospy.logerr("Error processing depth image: {}".format(e))

    def is_point_in_prohibited_zone(self, new_point, radius=8):
        """
        Check if the point is within a prohibited zone defined by previous points with a specified radius.
        Include checks against points currently on hold during the inspection.
        """
        # Check against previous semantic points
        for prev_point, _ in self.previous_semantic_points:
            if self.distance_between_points(prev_point, new_point) < radius:
                # rospy.loginfo(f"Point {new_point} is in a prohibited zone near previous point {prev_point}.")
                return True

        # Check against on-hold points to ensure they are not reprocessed
        for on_hold_point, _, _ in self.on_hold_points:
            if self.distance_between_points(on_hold_point, new_point) < radius:
                # rospy.loginfo(f"Point {new_point} is in a prohibited zone near on-hold point {on_hold_point}.")
                return True

        # rospy.loginfo(f"Point {new_point} is not in any prohibited zone.")
        return False

    def distance_between_points(self, point1, point2):
        """
        Compute the Euclidean distance between two points.
        """
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


    def publish_semantic_location(self, point, semantic_type, description=""):
        # rospy.loginfo(f"Currently in publishing........")
        # First, call the distance services to determine how far each robot is from the point
        try:
            my_response = self.my_distance_service(point=Point(x=point.x, y=point.y, z=point.z))
            other_response = self.other_distance_service(point=Point(x=point.x, y=point.y, z=point.z))
            
            # Log distances for debugging
            rospy.loginfo(f"Distance from my service: {my_response.distance}, Role: {my_response.role}")
            rospy.loginfo(f"Distance from other robot's service: {other_response.distance}, Role: {other_response.role}")

            # Choose the target publisher based on the distance 
            if other_response.distance > 15:
                target_pub = self.semantic_location_pub_to_explorer 
                rospy.loginfo(f"I will publish to the explorer........")

            else:
                rospy.loginfo(f"I will publish to the inspector ........")
                target_pub = self.semantic_location_pub_to_inspector

            # Publishing logic
            self.object_count += 1
            rospy.loginfo(f"object_count {self.object_count})")
            rospy.loginfo(f"World Frame Point for {semantic_type}: ({point.x}, {point.y}, {point.z})")
            self.publish_object_count()  # Publish each time an object is detected

            semantic_point_msg = SemanticPoint()
            semantic_point_msg.header.stamp = rospy.Time.now()
            semantic_point_msg.header.frame_id = "world"
            semantic_class_msg = SemanticClass()
            semantic_class_msg.value = semantic_type
            semantic_point_msg.type = semantic_class_msg
            semantic_point_msg.point = Point32(x=point.x, y=point.y, z=point.z)
            target_pub.publish(semantic_point_msg)
            self.publish_circle_marker(point)

            self.previous_semantic_points.append((point, description))
            self.publish_previous_semantic_points()

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call distance service: {e}")


    def publish_previous_semantic_points(self):
        for point, description in self.previous_semantic_points:
            semantic_point_msg = SemanticPoint()
            semantic_point_msg.header.stamp = rospy.Time.now()
            semantic_point_msg.header.frame_id = "world"
            semantic_point_msg.type.value = 5  # Ensure this is set correctly
            semantic_point_msg.point = Point32(x=point.x, y=point.y, z=point.z)
            self.previous_semantic_pub.publish(semantic_point_msg)

    def other_semantic_callback(self, msg):
        point = Point(msg.point.x, msg.point.y, msg.point.z)
        description = msg.type.value
        if not self.is_point_in_prohibited_zone(point, radius=0):
            self.previous_semantic_points.append((point, description))

    def process_object(self, depth_image, roi, class_name):
        center_x = int(roi.x_offset + roi.width / 2)
        center_y = int(roi.y_offset + roi.height / 2) 
        depth_value = depth_image[center_y, center_x] 
        if depth_value > 0:
            # rospy.loginfo(f"Depth at center of {class_name}: {depth_value}")
            point_world_frame , semantic_type = self.calculate_3d_position(depth_value, center_x, center_y, class_name)  # Pass class_name to the next method
            if self.previous_semantic_points == [] :
                    rospy.loginfo(" no previous semantic ... publishing the first!!!")
                    self.publish_semantic_location(point_world_frame, semantic_type, class_name)
            elif not self.is_point_in_prohibited_zone(point_world_frame):
                    if self.on_hold_points == [] :
                        self.publish_semantic_location(point_world_frame, semantic_type, class_name)
                        rospy.loginfo(" i'm not in the prohibited_zone .. i'm ready to publish ")
                    else:
                        rospy.loginfo(" i'm not in the prohibited_zone .. but i'm workig ...added to the on_hold_points ")
                        self.on_hold_points.append((point_world_frame, semantic_type, class_name))                                     

            return point_world_frame
    
    
    def calculate_3d_position(self, depth, px, py, class_name):
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        X = depth  # Assuming depth is in millimeters
        Z = (py - cy) * X / fy
        Y = -(px - cx) * X / fx

        # Convert point from camera frame to world frame
        point_camera_frame = Point(X+2, Y, Z)   
        namespace = rospy.get_namespace().strip('/')
        frame_id = f"{namespace}/base_link"
        point_world_frame = self.transform_point_to_world_frame(point_camera_frame, frame_id)
        if point_world_frame:
            semantic_type = self.get_semantic_type(class_name)
            if semantic_type is not None:
                return point_world_frame , semantic_type

    
    def get_semantic_type(self, class_name):
        if class_name == "chair":
            return SemanticClass.kChair
        elif class_name == "table":
            return SemanticClass.kTable
        elif class_name == "car" or class_name == "truck":
            return SemanticClass.kCar  # Assuming you have a semantic type for cars/trucks
        return None

    def transform_point_to_world_frame(self, point, frame_id):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position = point
        pose_stamped.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform("world",  # Target frame
                                                        frame_id,  # Source frame
                                                        rospy.Time(0),  # Get the latest available
                                                        rospy.Duration(1.0))  # Timeout
            #print("transform between world and robot",transform)
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            #print("pose_transformed",pose_transformed)
            return pose_transformed.pose.position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")
            return None

    def segmentation_callback(self, msg):
        self.current_segmentation = msg

    def camera_info_callback(self, msg):
        self.camera_info = msg

if __name__ == "__main__":
    processor = SemanticDepthProcessor()
    rospy.spin()