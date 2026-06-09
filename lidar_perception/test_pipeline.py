import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

class PubNode(Node):
    def __init__(self):
        super().__init__('test_pipeline')
        self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = 10
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 160
        
        # some points in ROI (x [0, 50], y [-20, 20], z [-2, 3])
        import struct
        data = bytearray()
        for i in range(10):
            # x=10, y=0, z=0
            data.extend(struct.pack('fff', 10.0, 0.0, 0.0))
            data.extend(bytes(4)) # padding
        msg.data = data
        msg.is_dense = True
        self.pub.publish(msg)
        self.get_logger().info('Published test pointcloud to ROI filter')
        
rclpy.init()
node = PubNode()
try:
    rclpy.spin_once(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
