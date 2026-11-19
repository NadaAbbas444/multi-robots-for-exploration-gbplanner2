# mask_rcnn_ros

Based on https://github.com/ethz-asl/mask_rcnn_ros (commit 5baaa44056b54d84ca56f4110f70c27df240627f),
which wraps https://github.com/matterport/Mask_RCNN (MIT, see LICENSE.Mask_R-CNN).
Modified for this project: multi-UAV detector node (scripts/mask_rcnn_node_modified.py),
semantic object localisation (scripts/object_location_modified.py), semantic
messages, and TensorFlow 2.4 / Keras 2.4 compatibility.

The COCO weights (bags/mask_rcnn_coco.h5, 246 MB) are not stored in git;
workspace/setup_workspace.sh downloads them from
https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
