from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import rclpy
from rclpy.node import Node
import math

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.sub = self.create_subscription(PointCloud2, '/livox/points', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.angle_min = -math.pi
        self.angle_max =  math.pi
        self.angle_increment = math.radians(0.25)
        self.range_min = 0.1
        self.range_max = 40.0
        self.z_min = 0.1   # m
        self.z_max = 0.15   # m

    def cb(self, msg):
        scan = LaserScan()
        scan.header.frame_id = "laser"
        
        # --- Thêm timestamp đồng bộ với Livox nhưng có offset nhỏ ---
        if not hasattr(self, 'scan_seq'):
            self.scan_seq = 0
        self.scan_seq += 1

        t = rclpy.time.Time.from_msg(msg.header.stamp)
        t = t + rclpy.duration.Duration(nanoseconds=self.scan_seq*1000)  # offset 1 µs
        scan.header.stamp = t.to_msg()
        # -----------------------------------------------------------

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        n = int((self.angle_max - self.angle_min)/self.angle_increment)
        ranges = np.full(n, np.inf)

        for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            x,y,z = p
            if z < self.z_min or z > self.z_max:
                continue
            r = math.sqrt(x*x+y*y)
            if self.range_min < r < self.range_max:
                angle = math.atan2(y,x)
                idx = int((angle - self.angle_min)/self.angle_increment)
                if 0 <= idx < n:
                    if r < ranges[idx]:
                        ranges[idx] = r

        scan.ranges = ranges.tolist()
        self.pub.publish(scan)

def main():
    rclpy.init()
    node = CloudToScan()
    rclpy.spin(node)
    rclpy.shutdown()
