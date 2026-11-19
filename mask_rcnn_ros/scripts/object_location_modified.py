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
from nav_msgs.msg import Odometry  # Import for odometry messages

from std_srvs.srv import Trigger

class SemanticDepthProcessor:
    def __init__(self):
        rospy.init_node('semantic_depth_processor')
        self.bridge = CvBridge()
        self.namespace = rospy.get_namespace().rstrip('/')
        self.other_namespace = 'blue' if self.namespace.endswith('red') else 'red'
        # Minimum detection score to localise an object (was hard-coded 0.95,
        # which simulated renders rarely reach).
        self.min_score = rospy.get_param('~min_score', 0.6)
        self.object_depth_offset = rospy.get_param('~object_depth_offset', 1.0)
        self.camera_frame = rospy.get_param('~camera_frame', '')  # default <ns>/camera

        self.global_planner_service_available = False
        self.object_count = 0  # Counter for detected objects

        self.check_planner_service_availability()
        self.initialize_nodes()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.current_segmentation = None
        self.camera_info = None
        self.previous_semantic_points = []


    def initialize_nodes(self):
        # self.toggle_planning_client = rospy.ServiceProxy(self.namespace + '/gbplanner/toggle_planning', SetBool)
        self.depth_sub = rospy.Subscriber(self.namespace + "/mask_rcnn/depth_sync", Image, self.depth_callback)
        self.segmentation_sub = rospy.Subscriber(self.namespace + "/mask_rcnn/result", Result, self.segmentation_callback)
        self.camera_info_sub = rospy.Subscriber(self.namespace + "/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        # self.objects_counter_pub = rospy.Publisher(self.namespace +"/detected_objects_counter", Int32, queue_size=10)
        self.segment_vicinity_marker = rospy.Publisher(self.namespace + "/segment_vicinity", Marker, queue_size=10)
        self.previous_semantic_pub = rospy.Publisher(self.namespace +"/previous_semantic_points", SemanticPoint, queue_size=10)
        rospy.Subscriber("/" + self.other_namespace + "/previous_semantic_points", SemanticPoint, self.other_semantic_callback)
# Subscribe to odometry topics
        rospy.Subscriber("/" + self.other_namespace + "/odometry", Odometry, self.other_odometry_callback)
        rospy.Subscriber(self.namespace + "/odometry", Odometry, self.current_odometry_callback)


    def publish_circle_marker(self, point, radius=5.0):
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
        marker.color.a = 0.4  # 50% opacity
        marker.color.r = 1.0
        marker.color.g = 0.0  # Green color
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()  # 0 or rospy.Duration() means never to auto-delete

        # Publish the marker
        self.segment_vicinity_marker.publish(marker)
                # Print odometry data
        if self.current_odometry:
            rospy.loginfo("Current namespace odometry: %s", self.current_odometry)
        if self.other_odometry:
            rospy.loginfo("Other namespace odometry: %s", self.other_odometry)

    def current_odometry_callback(self, msg):
        self.current_odometry = msg

    def other_odometry_callback(self, msg):
        self.other_odometry = msg


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


    def handle_object_detection(self, depth_image, i, seg=None):
        seg = seg or self.current_segmentation
        class_name = seg.class_names[i]
        score = seg.scores[i]
        if class_name in ["car", "truck","bus","motorcycle"] and score > self.min_score:
            # print("class_name",class_name,"score",score)
            self.process_object(depth_image, seg.boxes[i], class_name)

    def depth_callback(self, msg):
        if not self.global_planner_service_available:
            rospy.logwarn("Service not available.")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Snapshot: the segmentation callback may replace it mid-loop.
            seg = self.current_segmentation
            if seg and seg.boxes:
                min_length = min(len(seg.boxes), len(seg.class_names), len(seg.scores))
                for i in range(min_length):
                    self.handle_object_detection(depth_image, i, seg)
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

        # rospy.loginfo(f"Point {new_point} is not in any prohibited zone.")
        return False

    def distance_between_points(self, point1, point2):
        """
        Compute the Euclidean distance between two points.
        """
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

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
        # Median depth over the central 40% of the box: robust to background
        # pixels at the exact centre (e.g. between a vehicle's wheels).
        hw, hh = max(1, int(roi.width * 0.2)), max(1, int(roi.height * 0.2))
        patch = depth_image[max(0, center_y - hh):center_y + hh + 1,
                            max(0, center_x - hw):center_x + hw + 1]
        valid = patch[np.isfinite(patch) & (patch > 0.1)]
        depth_value = float(np.median(valid)) if valid.size else 0.0
        if depth_value > 0:
            result = self.calculate_3d_position(depth_value, center_x, center_y, class_name)
            if result is None:
                return None
            point_world_frame, semantic_type = result
            if self.previous_semantic_points == [] :
                    # rospy.loginfo(" no previous semantic ... publishing the first!!!")
                    self.publish_circle_marker(point_world_frame)

                    self.previous_semantic_points.append((point_world_frame, 5))
                    self.publish_previous_semantic_points()

                    # self.publish_semantic_location(point_world_frame, semantic_type, class_name)
            elif not self.is_point_in_prohibited_zone(point_world_frame):
                    self.publish_circle_marker(point_world_frame)
                    self.previous_semantic_points.append((point_world_frame, 5))
                    self.publish_previous_semantic_points()
                                

            return point_world_frame
    
    
    def calculate_3d_position(self, depth, px, py, class_name):
        """Pinhole back-projection in the camera optical frame (<ns>/camera:
        x right, y down, z forward), then TF to world. The point is pushed
        ~object_depth_offset further along the viewing ray, from the visible
        surface toward the object's centre (replaces the old fixed +2 m)."""
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        z = depth + self.object_depth_offset
        point_camera_frame = Point((px - cx) * z / fx, (py - cy) * z / fy, z)
        frame_id = self.camera_frame or f"{rospy.get_namespace().strip('/')}/camera"
        point_world_frame = self.transform_point_to_world_frame(point_camera_frame, frame_id)
        semantic_type = self.get_semantic_type(class_name)
        if point_world_frame is None or semantic_type is None:
            return None
        return point_world_frame, semantic_type

    def get_semantic_type(self, class_name):
        if class_name == "chair":
            return SemanticClass.kChair
        elif class_name == "table":
            return SemanticClass.kTable
        elif class_name == "car" or class_name == "truck" or class_name == "motorcycle" or class_name == "bus" :
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