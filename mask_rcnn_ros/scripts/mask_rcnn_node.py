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

# Local path to trained weights file
ROS_HOME = os.environ.get('ROS_HOME', os.path.join(os.environ['HOME'], '.ros'))
COCO_MODEL_PATH = '/home/nadaabbas/catkin2_ws/src/mask_rcnn_ros/bags/mask_rcnn_coco.h5'

# COCO Class names
# Index of the class in the list is its ID. For example, to get ID of
# the teddy bear class, use: CLASS_NAMES.index('teddy bear')
# CLASS_NAMES = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
#                'bus', 'train', 'truck', 'boat', 'traffic light',
#                'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
#                'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
#                'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
#                'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
#                'kite', 'baseball bat', 'baseball glove', 'skateboard',
#                'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
#                'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
#                'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
#                'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
#                'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
#                'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
#                'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
#                'teddy bear', 'hair drier', 'toothbrush']

CLASS_NAMES = ['car', 'truck']


CLASS_ID_MAP = {
    3: 'car',  # COCO ID for 'car'
    8: 'truck' , # COCO ID for 'truck'
}


class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    DETECTION_MAX_INSTANCES = 2
class MaskRCNNNode(object):
    def __init__(self):
        self.relevant_class_ids = [3, 8]  # COCO IDs for car and truck

        # Fetch and prepare the namespace
        self.namespace = rospy.get_param('~namespace', '').strip('/')
        
        if self.namespace:
            self.namespace = '/' + self.namespace  # Prepend a slash for proper topic formatting

        self._cv_bridge = CvBridge()
        self._visualization = rospy.get_param('~visualization', True)
        self._publish_rate = rospy.get_param('~publish_rate', 10)

        config = InferenceConfig()
        self._model = modellib.MaskRCNN(mode="inference", model_dir="", config=config)
        model_path = rospy.get_param('~model_path', COCO_MODEL_PATH)
        self._model.load_weights(model_path, by_name=True)
        self._class_names = rospy.get_param('~class_names', CLASS_NAMES)
        # self._class_colors = visualize.random_colors(len(CLASS_NAMES))

        self._last_rgb_msg = None
        self._last_depth_msg = None
        self._msg_lock = threading.Lock()

        # Setup publishers
        self._depth_pub = rospy.Publisher('~depth_sync', Image, queue_size=1)
        self._obj_prob_pub = rospy.Publisher('~object_probability', String, queue_size=1)
        
        # Setup subscribers
            
    def run(self):
        self._result_pub = rospy.Publisher('~result', Result, queue_size=1)
        self.vis_pub = rospy.Publisher('~visualization', Image, queue_size=1)
        sub_image =rospy.Subscriber(self.namespace + '/camera/color/image_raw', Image, self._image_callback, queue_size=1)
        sub_depth =rospy.Subscriber(self.namespace + '/camera/depth/image_raw', Image, self._depth_callback, queue_size=1)

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                rgb_msg = self._last_rgb_msg
                depth_msg = self._last_depth_msg
                self._last_rgb_msg = None
                self._last_depth_msg = None
                self._msg_lock.release()

                if rgb_msg and depth_msg:
                    np_image = self._cv_bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
                    results = self._model.detect([np_image], verbose=0)
                    result = results[0]

                    result_msg = self._build_result_msg(rgb_msg, result)
                    self._result_pub.publish(result_msg)
                    self._depth_pub.publish(depth_msg)

                    if self._visualization:
                        vis_image = self._visualize(result, np_image)
                        image_msg = self._cv_bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                        self.vis_pub.publish(image_msg)

            rate.sleep()

    def _image_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_rgb_msg = msg
            self._msg_lock.release()

    def _depth_callback(self, msg):
        with self._msg_lock:
            self._last_depth_msg = msg

    def _build_result_msg(self, msg, result):
        result_msg = Result()
        result_msg.header = msg.header
        for i, (y1, x1, y2, x2) in enumerate(result['rois']):
            class_id = result['class_ids'][i]
            
            # Check if the detected class is relevant
            if class_id in self.relevant_class_ids:
                box = RegionOfInterest()
                box.x_offset = int(x1)
                box.y_offset = int(y1)
                box.height = int(y2 - y1)
                box.width = int(x2 - x1)
                result_msg.boxes.append(box)

                result_msg.class_ids.append(class_id)
                class_name = CLASS_ID_MAP.get(class_id, "Unknown")  # Safer access with default
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

                # Publish object probability if relevant
                obj_prob_str = "{}: {:.2f}%".format(class_name, score * 100)
                self._obj_prob_pub.publish(obj_prob_str)

        return result_msg


    def _visualize(self, result, image):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        fig = Figure()
        canvas = FigureCanvasAgg(fig)
        axes = fig.gca()
        visualize.display_instances(image, result['rois'], result['masks'],
                                    result['class_ids'], CLASS_NAMES,
                                    result['scores'], ax=axes,
                                    class_colors=self._class_colors)
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

