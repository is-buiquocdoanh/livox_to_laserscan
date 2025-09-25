import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg

class LivoxToPointCloud(Node):
    def __init__(self):
        super().__init__('livox_to_pointcloud')
        self.sub = self.create_subscription(CustomMsg, '/livox/lidar', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/livox/points', 10)

    def cb(self, msg: CustomMsg):
        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id if msg.header.frame_id else "livox_frame"

        points = []
        for p in msg.points:
            # x,y,z + reflectivity as intensity
            points.append([p.x, p.y, p.z, float(p.reflectivity)])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, points)
        self.pub.publish(pc2_msg)

def main():
    rclpy.init()
    node = LivoxToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
