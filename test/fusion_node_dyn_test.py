import sys

sys.path.append('/home/hs-coburg.de/mou0308s/ros_ws/src/camera_lidar_fusion/test')

# from camlidar_fusion import fusion_node
from fusion_node import ReprojectionNode
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
import math
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
cv_image = "test_image.png"
msg_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

node = ReprojectionNode()

msg_scan = LaserScan()
msg_scan.ranges = [1, 1, 1, 1]
msg_scan.angle_min = -math.pi
msg_scan.angle_increment = 0.01246
node.latest_scan = msg_scan

msg_detections = Detection2DArray()
bb_x = BoundingBox2D()
bb_x.center.x = 300
bb_x.center.y = 150
bb_x.size_x = 50
bb_x.size_y = 50
msg_detections.detections[0].bbox = bb_x
msg_detections.detections[0].results[0].id = 1


def test_scan_callback():
    node.scan_callback(msg_scan.latest_scan)
    assert node.latest_scan == msg_scan.latest_scan


def test_polar2karth():
    r = 2.4
    i = 252
    x, y = node.polar2karth(r, i)
    assert x == 2.39912
    assert y == -0.00401


def test_detectnet_callback(msg_detections):
    node.detectnet_callback(msg_detections)
    assert node.bbx_list == [msg_detections.detections[0].results[0].id,
                             bb_x.center.x, bb_x.center.y,
                             bb_x.size_x / 2, bb_x.size_y / 2]


def test_image_callback():
    arr1 = np.zeros(10)
    arr2 = np.ones(5)
    scan = []

    list_1 = np.ravel(arr1).tolist()
    list_2 = np.ravel(arr2).tolist()
    scan.append(list_1)
    scan.append(list_2)
    scan.append(list_1)
    msg_scan.ranges = scan
    node.latest_scan = msg_scan
    node.image_callback(msg_image)

