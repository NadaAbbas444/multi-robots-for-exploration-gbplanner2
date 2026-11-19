#!/usr/bin/env python3
import os
import threading
from queue import Queue
import numpy as np

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import UInt8MultiArray , String

from mask_rcnn_ros import coco
from mask_rcnn_ros import utils
from mask_rcnn_ros import model as modellib
from mask_rcnn_ros import visualize
from mask_rcnn_ros.msg import Result

from skimage.transform import resize
#!/usr/bin/env python3
import os
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from mask_rcnn_ros.msg import Result

# Local path to trained weights file
ROS_HOME = os.environ.get('ROS_HOME', os.path.join(os.environ['HOME'], '.ros'))
COCO_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bags', 'mask_rcnn_coco.h5')

CLASS_NAMES = ['car', 'truck','motorcycle','Bus']
CLASS_ID_MAP = {3: 'car', 8: 'truck',4: 'motorcycle',6: 'bus'}

class InferenceConfig(coco.CocoConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    DETECTION_MAX_INSTANCES = 2

class MaskRCNNNode(object):
    def __init__(self):
        self.relevant_class_ids = [3, 8,4,6]  # COCO IDs for car and truck

        # One detector process serves every UAV in ~namespaces (default the
        # original red/blue pair); ~namespace is still accepted as the first.
        first = rospy.get_param('~namespace', 'red').strip('/')
        self.namespaces = [n.strip('/') for n in rospy.get_param('~namespaces', [first, 'blue' if first == 'red' else 'red'])]
        self.namespace = self.namespaces[0]

        self._cv_bridge = CvBridge()
        self._visualization = rospy.get_param('~visualization', True)
        self._publish_rate = rospy.get_param('~publish_rate', 10)

        config = InferenceConfig()
        # Keep more than a couple of instances (ceiling/rotor false positives
        # used to crowd out the vehicles) and accept the lower scores that
        # Gazebo renders get. Relevant classes are filtered afterwards.
        config.DETECTION_MAX_INSTANCES = int(rospy.get_param('~max_instances', 10))
        config.DETECTION_MIN_CONFIDENCE = float(rospy.get_param('~min_confidence', 0.5))
        self._model = modellib.MaskRCNN(mode="inference", model_dir="", config=config)
        model_path = rospy.get_param('~model_path', COCO_MODEL_PATH)
        self._model.load_weights(model_path, by_name=True)
        self._class_names = rospy.get_param('~class_names', CLASS_NAMES)

        self._last_rgb_msg = {ns: None for ns in self.namespaces}
        self._last_depth_msg = {ns: None for ns in self.namespaces}
        self._msg_lock = threading.Lock()

        self._depth_pub = {ns: rospy.Publisher(f'/{ns}/mask_rcnn/depth_sync', Image, queue_size=1) for ns in self.namespaces}
        self._obj_prob_pub = {ns: rospy.Publisher(f'/{ns}/mask_rcnn/object_probability', String, queue_size=1) for ns in self.namespaces}
        self._result_pub = {ns: rospy.Publisher(f'/{ns}/mask_rcnn/result', Result, queue_size=1) for ns in self.namespaces}
        self.vis_pub = {ns: rospy.Publisher(f'/{ns}/mask_rcnn/visualization', Image, queue_size=1) for ns in self.namespaces}
        rospy.loginfo("Mask R-CNN serving UAVs: %s", ", ".join(self.namespaces))

    def run(self):
        for ns in self.namespaces:
            rospy.Subscriber(f'/{ns}/camera/color/image_raw', Image, self._image_callback, callback_args=ns, queue_size=1)
            rospy.Subscriber(f'/{ns}/camera/depth/image_raw', Image, self._depth_callback, callback_args=ns, queue_size=1)

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            for ns in self.namespaces:
                self.process_images(ns)
            rate.sleep()

    def process_images(self, namespace):
        with self._msg_lock:
            rgb_msg = self._last_rgb_msg[namespace]
            depth_msg = self._last_depth_msg[namespace]
            self._last_rgb_msg[namespace] = None
            self._last_depth_msg[namespace] = None

        if rgb_msg and depth_msg:
            np_image = self._cv_bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            results = self._model.detect([np_image], verbose=0)
            result = results[0]

            result_msg = self._build_result_msg(rgb_msg, result, namespace)
            self._result_pub[namespace].publish(result_msg)
            self._depth_pub[namespace].publish(depth_msg)

            # if self._visualization:
            #     vis_image = self._visualize(result, np_image)
            #     image_msg = self._cv_bridge.cv2_to_imgmsg(vis_image, 'bgr8')
            #     self.vis_pub[namespace].publish(image_msg)

    def _image_callback(self, msg, namespace):
        rospy.logdebug(f"Received {namespace} image")
        with self._msg_lock:
            self._last_rgb_msg[namespace] = msg

    def _depth_callback(self, msg, namespace):
        with self._msg_lock:
            self._last_depth_msg[namespace] = msg

    def _build_result_msg(self, msg, result, namespace):
        result_msg = Result()
        result_msg.header = msg.header
        for i, (y1, x1, y2, x2) in enumerate(result['rois']):
            class_id = result['class_ids'][i]
            
            if class_id in self.relevant_class_ids:
                box = RegionOfInterest()
                box.x_offset = int(x1)
                box.y_offset = int(y1)
                box.height = int(y2 - y1)
                box.width = int(x2 - x1)
                result_msg.boxes.append(box)

                result_msg.class_ids.append(class_id)
                class_name = CLASS_ID_MAP.get(class_id, "Unknown")
                result_msg.class_names.append(class_name)

                score = result['scores'][i]
                result_msg.scores.append(score)

                mask = Image()
                mask.header = msg.header
                mask.height = result['masks'].shape[0]
                mask.width = result['masks'].shape[1]
                mask.encoding = "mono8"
                mask.is_bigendian = False
                mask.step = mask.width
                mask.data = (result['masks'][:, :, i] * 255).tobytes()
                result_msg.masks.append(mask)

                obj_prob_str = "{}: {:.2f}%".format(class_name, score * 100)
                self._obj_prob_pub[namespace].publish(obj_prob_str)

        return result_msg

    def _visualize(self, result, image):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        fig = Figure()
        canvas = FigureCanvasAgg(fig)
        axes = fig.gca()
        visualize.display_instances(image, result['rois'], result['masks'],
                                    result['class_ids'], CLASS_NAMES,
                                    result['scores'], ax=axes)
        fig.tight_layout()
        canvas.draw()
        result = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')

        _, _, w, h = fig.bbox.bounds
        result = result.reshape((int(h), int(w), 3))
        return result

def main():
    rospy.init_node('mask_rcnn')

    node = MaskRCNNNode()
    node.run()

if __name__ == '__main__':
    main()

