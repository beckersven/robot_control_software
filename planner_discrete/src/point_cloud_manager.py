#! /usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from tf2_geometry_msgs import PointStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
from threading import Lock
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse, SetBoolRequest
import struct

class PointCloudManager:
    def __init__(self):
        rospy.init_node("point_cloud_manager")
        self.stitched_point_cloud = PointCloud()
        self.buffer = tf2_ros.Buffer()
        self.stitch = True
        tf2_ros.TransformListener(self.buffer)
        self.tf_timeout = rospy.Duration(rospy.get_param("/coppelia_config/sim_dt") * 3)
        self.process_mutex = Lock()
        self.stitched_point_cloud_pub = rospy.Publisher("/stitched_pointcloud", PointCloud, queue_size=1, latch=True)
        self.stitched_point_cloud.header.frame_id = "world"
        self.stitched_point_cloud.channels.append(ChannelFloat32())
        self.stitched_point_cloud.channels[0].name = "rgb"
        self.current_point_cloud_pub = rospy.Publisher("/processed_scan_data", PointCloud, queue_size=1, latch=True)
        rospy.Subscriber("/raw_scan_data", Float32MultiArray, self.process_scan)
        rospy.Service("/clear_stitched_point_cloud", Trigger, self.clear_stitched_point_cloud_callback)
        rospy.Service("/switch_stitch_mode", SetBool, self.switch_stitch_mode)

    def switch_stitch_mode(self, req):
        assert isinstance(req, SetBoolRequest)
        with self.process_mutex:
            self.stitch = req.data
        return SetBoolResponse(success=True, message="{}activated stitching".format("de" * int(req.data)))
        

    def clear_stitched_point_cloud_callback(self, req):
        with self.process_mutex:
            self.stitched_point_cloud.points = []
            self.stitched_point_cloud.channels[0].values = []
        return TriggerResponse(success=True, message="Cleared the stitched point cloud")


    def process_scan(self, raw_data):
        assert isinstance(raw_data, Float32MultiArray)
        stamp = rospy.Time(raw_data.data[0])
        raw_data.data = raw_data.data[1:]
        current_point_cloud = PointCloud()
        current_point_cloud.header.frame_id = "laser_emitter_frame"
        current_point_cloud.header.stamp = stamp
        current_point_cloud.channels.append(ChannelFloat32())
        current_point_cloud.channels[0].name = "rgb"
        if len(raw_data.data) / 6 % 1 != 0:
            rospy.logerr("Received wrongly shaped raw laser scan data")
            return
        with self.process_mutex:
            for i in range(len(raw_data.data) / 6):
                new_point = PointStamped()
                new_point.header.frame_id = "laser_emitter_frame"
                new_point.header.stamp = stamp
                new_point.point = Point32(*raw_data.data[6 * i: 6 * i + 3])
                current_point_cloud.points.append(new_point.point)
                current_point_cloud.channels[0].values.append(struct.unpack('f', struct.pack('i', 0xff0000))[0])
                if self.stitch:
                    try:
                        new_point = self.buffer.transform(new_point, "world", self.tf_timeout)
                    except tf2_ros.ExtrapolationException as e:
                        if "extrapolation into the past" in e.message:
                            rospy.logwarn("Tried to read too old tf-values. Ignore this if this happens only once.")
                            break
                    self.stitched_point_cloud.points.append(new_point.point) 
                    self.stitched_point_cloud.channels[0].values.append(struct.unpack('f', struct.pack('i', 0x00ff00))[0])   
            self.current_point_cloud_pub.publish(current_point_cloud)
            self.stitched_point_cloud.header.stamp = stamp
            self.stitched_point_cloud_pub.publish(self.stitched_point_cloud)


if __name__ == "__main__":
    p = PointCloudManager()
    rospy.spin()