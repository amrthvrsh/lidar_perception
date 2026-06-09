#!/usr/bin/env python3
"""
lidar_projection_node.py
------------------------
Subscribes to:
  - Raw camera image
  - Camera info  (one-shot)
  - /lidar/clusters  (PointCloud2, intensity = cluster_id)
  - /camera/detections  (CameraDetectionArray from YOLO)

For every YOLO detection, finds cluster points whose 2D projection
falls inside the bounding box and draws them on the image.
Points outside any YOLO box are discarded.

Publishes:
  - /fusion/projected_image  (Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener

from lidar_perception.msg import CameraDetectionArray


# ── Colour palette (BGR) per cluster id ───────────────────────────────────────
CLUSTER_COLORS = [
    (0,   255, 255),  # yellow
    (0,   165, 255),  # orange
    (255, 0,   255),  # magenta
    (0,   255, 0),    # green
    (255, 100, 0),    # blue-ish
    (0,   100, 255),  # orange-red
    (100, 255, 100),  # light green
    (255, 0,   100),  # purple
]


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion → 3×3 rotation matrix (no extra deps)."""
    return np.array([
        [1-2*(qy*qy+qz*qz),  2*(qx*qy-qz*qw),    2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),    1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),    2*(qy*qz+qx*qw),    1-2*(qx*qx+qy*qy)],
    ])


def read_xyz_intensity(msg: PointCloud2):
    """
    Fast numpy extraction of (x, y, z, intensity) from a PointCloud2.
    Assumes PointXYZI layout produced by PCL (x@0, y@4, z@8, intensity@16).
    Falls back to field-offset lookup if layout differs.
    """
    fields = {f.name: f for f in msg.fields}
    offsets = {k: fields[k].offset for k in ('x', 'y', 'z', 'intensity') if k in fields}

    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)

    def extract(off):
        col = raw[:, off: off + 4].copy()
        return col.view(np.float32).flatten()

    x = extract(offsets['x'])
    y = extract(offsets['y'])
    z = extract(offsets['z'])
    intensity = extract(offsets['intensity']) if 'intensity' in offsets else np.zeros(len(x))

    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid], intensity[valid]


class LidarProjectionNode(Node):

    def __init__(self):
        super().__init__('lidar_projection_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('image_topic',        '/sensing/camera/left/image')
        self.declare_parameter('camera_info_topic',  '/sensing/camera/left/camera_info')
        self.declare_parameter('clusters_topic',     '/lidar/clusters')
        self.declare_parameter('detections_topic',   '/camera/detections')
        self.declare_parameter('output_topic',       '/fusion/projected_image')
        self.declare_parameter('camera_frame',       'camera_left')
        self.declare_parameter('point_radius',       4)
        self.declare_parameter('sync_slop',          0.15)  # seconds

        image_topic       = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        clusters_topic    = self.get_parameter('clusters_topic').value
        detections_topic  = self.get_parameter('detections_topic').value
        output_topic      = self.get_parameter('output_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.radius       = self.get_parameter('point_radius').value
        slop              = self.get_parameter('sync_slop').value

        # ── State ────────────────────────────────────────────────────────────
        self.K      = None   # 3×3 intrinsic matrix
        self.T_lidar_to_cam = None  # 4×4 cached transform
        self.lidar_frame    = None
        self.bridge = CvBridge()

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Camera info (one-shot) ────────────────────────────────────────────
        self.cam_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic,
            self._camera_info_cb, qos_profile_sensor_data)

        # ── Synchronised subscriptions ────────────────────────────────────────
        self.img_sub  = message_filters.Subscriber(
            self, Image, image_topic,
            qos_profile=qos_profile_sensor_data)
        self.pts_sub  = message_filters.Subscriber(
            self, PointCloud2, clusters_topic,
            qos_profile=qos_profile_sensor_data)
        self.det_sub  = message_filters.Subscriber(
            self, CameraDetectionArray, detections_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.img_sub, self.pts_sub, self.det_sub],
            queue_size=10, slop=slop)
        self.sync.registerCallback(self._callback)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'LiDAR Projection Node ready.\n'
            f'  Clusters : {clusters_topic}\n'
            f'  Detections: {detections_topic}\n'
            f'  Output   : {output_topic}')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f'Camera intrinsics loaded: '
                f'fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def _get_transform(self, lidar_frame: str):
        """Lookup and cache lidar→camera TF as a 4×4 numpy matrix."""
        if self.T_lidar_to_cam is not None and self.lidar_frame == lidar_frame:
            return self.T_lidar_to_cam

        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, lidar_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed ({lidar_frame}→{self.camera_frame}): {e}',
                                   throttle_duration_sec=2.0)
            return None

        t  = tf.transform.translation
        q  = tf.transform.rotation
        R  = quat_to_rot(q.x, q.y, q.z, q.w)
        T  = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = [t.x, t.y, t.z]

        self.T_lidar_to_cam = T
        self.lidar_frame    = lidar_frame
        self.get_logger().info(f'TF cached: {lidar_frame} → {self.camera_frame}')
        return T

    def _callback(self, img_msg: Image, pts_msg: PointCloud2,
                  det_msg: CameraDetectionArray):

        if self.K is None:
            self.get_logger().warn('No camera_info yet.', throttle_duration_sec=2.0)
            return

        # 1. Get transform
        T = self._get_transform(pts_msg.header.frame_id)
        if T is None:
            return

        # 2. Decode image
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        h, w = frame.shape[:2]

        # 3. Extract cluster points
        x, y, z, intensity = read_xyz_intensity(pts_msg)
        if x.size == 0:
            self._publish(frame, img_msg)
            return

        cluster_ids = intensity.astype(np.int32)

        # 4. Transform to camera frame  (vectorised)
        ones = np.ones(len(x))
        pts_lidar = np.vstack([x, y, z, ones])   # 4×N
        pts_cam   = T @ pts_lidar                  # 4×N

        # 5. Keep only points in front of camera
        front = pts_cam[2] > 0.1
        pts_cam     = pts_cam[:, front]
        cluster_ids = cluster_ids[front]
        depths      = pts_cam[2]

        if pts_cam.shape[1] == 0:
            self._publish(frame, img_msg)
            return

        # 6. Project to pixel coordinates
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        u = (fx * pts_cam[0] / depths + cx).astype(np.int32)
        v = (fy * pts_cam[1] / depths + cy).astype(np.int32)

        # 7. For each YOLO detection, draw matching cluster points
        output = frame.copy()
        detections = det_msg.detections

        if not detections:
            self._publish(output, img_msg)
            return

        drawn_any = False
        for det in detections:
            x0 = max(0, int(det.xmin))
            y0 = max(0, int(det.ymin))
            x1 = min(w - 1, int(det.xmax))
            y1 = min(h - 1, int(det.ymax))

            # Points whose projection falls inside this bbox
            inside = (u >= x0) & (u <= x1) & (v >= y0) & (v <= y1)
            if not np.any(inside):
                continue

            drawn_any = True

            # Draw bounding box
            cv2.rectangle(output, (x0, y0), (x1, y1), (0, 255, 0), 2)
            label = f"{det.class_label} {det.confidence*100:.0f}%"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(output, (x0, y0 - th - 8), (x0 + tw + 4, y0), (0, 200, 0), -1)
            cv2.putText(output, label, (x0 + 2, y0 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            # Draw cluster points coloured by depth
            u_in  = u[inside]
            v_in  = v[inside]
            d_in  = depths[inside]
            cid_in = cluster_ids[inside]

            for pi in range(len(u_in)):
                # Colour: close = warm red, far = cool blue (jet-like)
                depth_norm = float(np.clip(d_in[pi] / 30.0, 0.0, 1.0))
                r_c = int(255 * max(0.0, 1.0 - 2.0 * depth_norm))
                g_c = int(255 * max(0.0, 1.0 - abs(2.0 * depth_norm - 1.0)))
                b_c = int(255 * max(0.0, 2.0 * depth_norm - 1.0))
                cv2.circle(output, (u_in[pi], v_in[pi]),
                           self.radius, (b_c, g_c, r_c), -1)

            # Cluster count annotation (bottom-left of bbox)
            n_pts = int(np.sum(inside))
            cv2.putText(output, f'{n_pts}pts', (x0, y1 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

        self._publish(output, img_msg)

    def _publish(self, frame, ref_msg):
        try:
            out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            out.header = ref_msg.header
            self.pub.publish(out)
        except CvBridgeError as e:
            self.get_logger().error(f'Publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
