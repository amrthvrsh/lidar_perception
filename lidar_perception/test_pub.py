import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class PubNode(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(PointCloud2, '/lidar/points_downsampled', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = 20
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 320
        msg.data = bytearray([0]*320)
        msg.is_dense = True
        self.pub.publish(msg)
        self.get_logger().info('Published test pointcloud')
        
rclpy.init()
node = PubNode()
try:
    rclpy.spin_once(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
