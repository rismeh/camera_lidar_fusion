#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Detection2DArray
from geometry_msgs.msg import PoseWithCovariance, Pose, Pose2D
from .laser_geometry_.laser_geometry import LaserProjection
from .laser_geometry_ import point_cloud2_ as pc2
from marvelmind_interfaces.msg import HedgePos, HedgePosA


def extract(point):
    return [point[0], point[1], point[2]]


class ReprojectionNode(Node):
    def __init__(self):
        super().__init__('reprojection')
        self.latest_scan = None
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)  # set the reliability of the subscription to best effort

        self.calib_file = '/home/af/ros2_ws/src/camlidar_fusion/resource/data/calibration_result_02_06.txt'
        self.config_file = '/home/af/ros2_ws/src/camlidar_fusion/resource/data/config.yaml'

        self.laser_point_radius = 2
        self.bridge = CvBridge()
        self.img = None
        self.lp = LaserProjection()

        self.detectnet_flage = False
        self.counter = 0

        with open(self.calib_file, 'r') as file:
            data = file.read().split()
            qx_value = float(data[0])
            qy_value = float(data[1])
            qz_value = float(data[2])
            qw_value = float(data[3])
            tx_value = float(data[4])
            ty_value = float(data[5])
            tz_value = float(data[6])
        self.q = Quaternion(qw_value, qx_value, qy_value, qz_value).transformation_matrix
        self.q[0, 3] = tx_value
        self.q[1, 3] = ty_value
        self.q[2, 3] = tz_value
        self.get_logger().info("Extrinsic parameter - camera to laser")
        self.get_logger().info(str(self.q))
        self.tvec = self.q[:3, 3]
        self.rot_mat = self.q[:3, :3]
        self.rot_mat_1 = np.linalg.inv(self.rot_mat)
        self.rvec, _ = cv2.Rodrigues(self.rot_mat)
        self.transf_1 = np.ones((4, 4))
        self.transf_1[:3, :3] = self.rot_mat_1
        self.transf_1[:3, 3] = -1 * np.dot(self.rot_mat_1, self.tvec)
        self.transf_1[3, :3] = np.zeros(3)
        self.get_logger().info(f'transfor: {self.transf_1}')

        with open(self.config_file, 'r') as file:
            file.readline()
            config = yaml.load(file, Loader=yaml.FullLoader)
            self.lens = config['lens']
            fx_value = float(config['fx'])
            fy_value = float(config['fy'])
            cx_value = float(config['cx'])
            cy_value = float(config['cy'])
            k1_value = float(config['k1'])
            k2_value = float(config['k2'])
            p1_value = float(config['p1/k3'])
            p2_value = float(config['p2/k4'])
        self.k_matrix = np.matrix([[fx_value, 0.0, cx_value],
                            [0.0, fy_value, cy_value],
                            [0.0, 0.0, 1.0]])
        self.k_1 = np.linalg.inv(self.k_matrix)
        self.d_vec = np.array([k1_value, k2_value, p1_value, p2_value])
        self.get_logger().info("Camera parameters")
        self.get_logger().info("Lens = %s" % self.lens)
        self.get_logger().info("K =")
        self.get_logger().info(str(self.k_matrix))
        self.get_logger().info("D =")
        self.get_logger().info(str(self.d_vec))

        self.pub = self.create_publisher(Image, '/reprojection', 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile)
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.detectnet_sub = self.create_subscription(Detection2DArray, '/detectnet/detections',
                                                      self.detectnet_callback, 10)
        self.publisher_ = self.create_publisher(HedgePosA, 'obj_center', 10)
        self.publisher_ = self.create_publisher(HedgePosA, 'Kante_R', 10)
        self.publisher_ = self.create_publisher(HedgePosA, 'Kante_L', 10)

    def scan_callback(self, scan):
        self.latest_scan = scan

    def image_callback(self, image):
        if hasattr(self, 'latest_scan'):
            diff = abs(image.header.stamp.nanosec - self.latest_scan.header.stamp.nanosec)
            self.get_logger().info("diff between laser frame and image frame: %d ns" % diff)
            index = []
            for i in range(len(self.latest_scan.ranges)):
                if self.latest_scan.ranges[i]:
                    index.append(i)
            self.img = self.bridge.imgmsg_to_cv2(image)
            cloud = self.lp.projectLaser(scan_in=self.latest_scan)
            points = pc2.read_points(cloud)
            objpoints = np.array(list(map(extract, points)))
            xyz_lidar = objpoints
            self.get_logger().info(f'Total scans: {len(self.latest_scan.ranges)}')
            self.get_logger().info(f'scans in: {len(index)}')
            self.get_logger().info(f'objponts: {objpoints.shape}')
            Z = self.get_z(self.q, objpoints, self.k_matrix)
            filt = np.argwhere(Z > 0)
            index = np.array(index)
            index = np.ravel(index[filt]).tolist()
            objpoints = objpoints[Z > 0]

            if self.lens == 'pinhole':
                img_points, _ = cv2.projectPoints(objpoints, self.rvec, self.tvec, self.k_matrix,
                                                  self.d_vec)  # Projects 3D points to an image plane.
            img_points = np.squeeze(img_points)

            ID = []
            obj_class = []
            if hasattr(self, 'bbx_list'):
                if self.detectnet_flage:
                    for detection in self.bbx_list:
                        obj_i = []
                        obj_class_i = []
                        bb_up = detection[2] - detection[4]
                        bb_down = detection[2] + detection[4]
                        bb_reight = detection[1] + detection[3]
                        bb_left = detection[1] - detection[3]
                        cv2.rectangle(img, (int(detection[1] - detection[3]), int(detection[2] - detection[4])),
                                      (int(detection[1] + detection[3]), int(detection[2] + detection[4])), (255, 0, 0),
                                      2)
                        for i in range(len(img_points)):
                            if img_points[i][1] > bb_up and img_points[i][1] < bb_down and img_points[i][
                                0] > bb_left and img_points[i][0] < bb_reight:
                                try:
                                    cv2.circle(self.img, (int(round(img_points[i][0])), int(round(img_points[i][1]))),
                                               self.laser_point_radius, (255, 0, 0), 1)
                                    obj_i.append(i)
                                    obj_class_i.append(detection[0])
                                except OverflowError:
                                    continue
                        if obj_i:
                            ID.append(obj_i)
                            obj_class.append(obj_class_i)
                    self.detectnet_flage = False
                else:
                    for i in range(len(img_points)):
                        try:
                            cv2.circle(self.img, (int(round(img_points[i][0])), int(round(img_points[i][1]))),
                                       self.laser_point_radius, (0, 255, 0), 1)
                        except OverflowError:
                            continue

            self.get_logger().info(f'N of detected objects: {len(ID)}')

            index = np.array(index)
            objs_beams_index = []
            if ID:
                for obj in ID:
                    arr = np.ravel(index[obj])
                    objs_beams_index.append(arr.tolist())
                objs_beams = []
                for obj_i in objs_beams_index:
                    beams_i = [self.latest_scan.ranges[i] for i in obj_i]
                    objs_beams.append(beams_i)

                obj_beams_filtered = []
                objs_beams_filtered_index = []
                for k in range(len(objs_beams)):
                    beams = objs_beams[k]
                    beams_index = objs_beams_index[k]
                    min_b = min(beams)
                    filtered = []
                    filtered_index = []
                    for i in range(len(beams)):
                        if beams[i] < (min_b + 0.5):
                            filtered.append(beams[i])
                            filtered_index.append(beams_index[i])
                    obj_beams_filtered.append(filtered)
                    objs_beams_filtered_index.append(filtered_index)

                for i in range(len(obj_beams_filtered)):
                    obj_b = obj_beams_filtered[i]
                    obj_indexes = objs_beams_filtered_index[i]

                    min_b = min(obj_b)
                    index_b = obj_b.index(min_b)
                    min_b_index = obj_indexes[index_b]

                    p_r_beam = obj_b[0]
                    p_r_beam_index = obj_indexes[0]

                    p_l_beam = obj_b[len(obj_b) - 1]
                    p_l_beam_index = obj_indexes[len(obj_indexes) - 1]

                    hinderniss_id = obj_class[i][0]
                    c_x, c_y = self.polar2karth(min_b, min_b_index)
                    kante_r_x, kante_r_y = self.polar2karth(p_r_beam, p_r_beam_index)
                    kante_l_x, kante_l_y = self.polar2karth(p_l_beam, p_l_beam_index)

                    msg_obj = HedgePosA()
                    msg_kante_r = HedgePosA()
                    msg_kante_l = HedgePosA()

                    msg_obj.address = self.counter
                    msg_obj.x_m = c_x
                    msg_obj.y_m = c_y
                    msg_obj.z_m = float(hinderniss_id)

                    msg_kante_r.address = self.counter
                    msg_kante_r.x_m = kante_r_x
                    msg_kante_r.y_m = kante_r_y
                    msg_kante_r.z_m = float(hinderniss_id)

                    msg_kante_l.address = self.counter
                    msg_kante_l.x_m = kante_l_x
                    msg_kante_l.y_m = kante_l_y
                    msg_kante_l.z_m = float(hinderniss_id)

                    self.publisher_.publish(msg_obj)
                    self.publisher_.publish(msg_kante_r)
                    self.publisher_.publish(msg_kante_l)

                    self.counter += 1

                    self.get_logger().info(f'hinderniss ID: {hinderniss_id}')
                    self.get_logger().info(f'Center: ({c_x},{c_y})')
                    self.get_logger().info(f'K_R: ({kante_r_x},{kante_r_y})')
                    self.get_logger().info(f'K_L: ({kante_l_x},{kante_l_y})')

            self.pub.publish(self.bridge.cv2_to_imgmsg(self.img, 'rgb8'))

    def get_z(self, T_cam_world, T_world_pc, K):
        R = T_cam_world[:3, :3]
        t = T_cam_world[:3, 3]
        proj_mat = np.dot(K, np.hstack((R, t[:, np.newaxis])))
        xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
        xy_hom = np.dot(proj_mat, xyz_hom.T).T
        z = xy_hom[:, -1]
        z = np.asarray(z).squeeze()
        return z

    def polar2karth(self, r, i):
        angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        return x, y

    def detectnet_callback(self, msg):
        self.get_logger().info(f'detectnet recieded')
        self.detectnet_flage = True
        detections_arr = msg.detections
        self.bbx_list = []
        for detection in detections_arr:
            bb_id = detection.results[0].id
            if bb_id == '\x01':
                bb_id = 1
            elif bb_id == '\x02':
                bb_id = 2
            elif bb_id == '\x03':
                bb_id = 3
            elif bb_id == '\x04':
                bb_id = 4
            elif bb_id == '\x05':
                bb_id = 5
            bb_center_x = detection.bbox.center.x
            bb_center_y = detection.bbox.center.y
            bb_size_x = detection.bbox.size_x
            bb_size_y = detection.bbox.size_y
            self.bbx_list.append([bb_id, bb_center_x, bb_center_y, bb_size_x / 2, bb_size_y / 2])

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = ReprojectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
