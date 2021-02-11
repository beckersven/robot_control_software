#! /usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from tf2_geometry_msgs import PointStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
from threading import Lock
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolResponse, SetBoolRequest
import struct
from agiprobot_measurement.sensor_model import SensorModel



class PointCloudManager:
    """
    Class for processing and evaluation of the raw scan-data published to ROS by CoppeliaSim.        
    """
    def __init__(self):
        # Initialize node
        rospy.init_node("point_cloud_manager")

        # Create and configure class member containing all the points received in the "world" frame
        self.stitched_point_cloud = PointCloud()
        self.stitched_point_cloud.header.frame_id = "world"
        self.stitched_point_cloud.channels.append(ChannelFloat32())
        self.stitched_point_cloud.channels[0].name = "rgb"
        
        # Flag indicating whether to store points to the stitched cloud or not
        self.stitch = False
        
        # Threading-Lock to prevent multiple accesses on self.stitched_point_cloud at the same moment ("race condition")
        self.stitch_mutex = Lock()
        
        # Required for uncertainty evaluation of incoming data:
        self.sensor_model = SensorModel(rospy.get_param("/sensor_model_parameters"))
        
        # Create publishers (one for the currently received laser line, one for the total stitched point cloud)
        self.stitched_point_cloud_pub = rospy.Publisher("stitched_point_cloud", PointCloud, queue_size=1, latch=True)
        self.current_scan_line_pub = rospy.Publisher("current_scan_line", PointCloud, queue_size=1, latch=True)
        
        # Create service callbacks to clear the stitched point cloud or to turn on/off stitching in general on command
        rospy.Service("clear_stitched_point_cloud", Trigger, self.clear_stitched_point_cloud_callback)
        rospy.Service("switch_stitch_mode", SetBool, self.set_stitch_mode_callback)

        # Listen to the scan-data published by CoppeliaSim
        rospy.Subscriber("/raw_scan_data", Float32MultiArray, self.process_scan_callback)

    def set_stitch_mode_callback(self, req):
        """
        Turns on or off point-stitching dependent on the boolean req.data
        
        :param req: Request with a boolean flag indicating whether to enable (true) or disable (false) stitching mode
        :type req: std_srvs/SetBoolRequest
        :returns: Response about the success of dis-/enabling of stitching and a message about the performed mode-set
        :rtype: std_srvs/SetBoolResponse
        """
        assert isinstance(req, SetBoolRequest)
        with self.stitch_mutex:
            self.stitch = req.data
        return SetBoolResponse(success=True, message="{}activated stitching".format("de" * int(not req.data)))
        

    def clear_stitched_point_cloud_callback(self, req):
        """
        Clears the class member stitched_point_cloud
        
        :param req: - (contains no information)
        :type req: std_srvs/TriggerRequest
        :returns: Response about the success of clearing the stitched point cloud
        :rtype: std_srvs/TriggerResponse
        """
        assert isinstance(req, TriggerRequest)
        with self.stitch_mutex:
            self.stitched_point_cloud.points = []
            self.stitched_point_cloud.channels[0].values = []
        return TriggerResponse(success=True, message="Cleared the stitched point cloud")


    def process_scan_callback(self, raw_data):
        """Decodes scanner-data from CoppeliaSim and publishes the current scan-line. If stitching is enabled,
        the points of the scan-line will be stored in the corresponding class member and the stitched point cloud becomes published.
        
        
        :param raw_data: Raw-data from CoppeliaSim containing coordinates and uncertainty-relevant information of each measured point as well as a ROS time stamp
        :type raw_data: std_msgs/Float32MultiArray
        :returns: None
        :rtype: NoneType
        """

        # The scan-data is decoded as an array of 32-Bit floats with length 1 + 6 * n (n is integer) according to:
        # First entry is time-stamp (which is important to map the sensors point cloud to the correct transform),
        # subsequent are batches of 6 values, each representing one measured point by CoppliaSim. In each batch,
        # the first 3 values represent coordinates of the point in the sensor frame and the 
        # last 3 values are the uncertainty-critical characteristics z (in mm), gamma (in rad) and theta (in rad).

        assert isinstance(raw_data, Float32MultiArray)
        
        # Ensure correct shape
        if (len(raw_data.data) - 1) / 6 % 1 != 0:
            rospy.logerr("Received wrongly shaped raw laser scan data")
            return
        
        # Read out timestamp
        stamp = rospy.Time(raw_data.data[0])
        raw_data.data = raw_data.data[1:]
        
        # Contains currently scanned points:
        current_scan_line = PointCloud()
        current_scan_line.header.frame_id = "world"
        current_scan_line.header.stamp = stamp
        current_scan_line.channels.append(ChannelFloat32())
        current_scan_line.channels[0].name = "rgb"
        
        with self.stitch_mutex:
            # Add every batch (= 1 scan point) to the pointcloud(s)
            for i in range(len(raw_data.data) / 6):
                # Geometrical tranform
                new_point = PointStamped()
                new_point.header.frame_id = "laser_emitter_frame"
                new_point.header.stamp = stamp
                new_point.point = Point32(*raw_data.data[6 * i: 6 * i + 3])
                current_scan_line.points.append(new_point.point)

                # Uncertainty evaluation: Color of the extracted point changes linearly from green to red as uncertainty-score
                # declines from 1 to 0 (see http://wiki.ros.org/rviz/DisplayTypes/PointCloud at section 0.0.2)
                score = self.sensor_model.evaluate_score(*raw_data.data[6 * i + 3:6 * i + 5])
                print(raw_data.data[4])
                color_hex = struct.unpack('f', struct.pack('i', (int(0xff * (1 - score)) << 16) + (int(0xff * score) << 8)))[0]
                current_scan_line.channels[0].values.append(color_hex)
            if self.stitch:                    
                self.stitched_point_cloud.points.extend(current_scan_line.points) 
                self.stitched_point_cloud.channels[0].values.extend(current_scan_line.channels[0].values)
            
            # Send out the results   
            self.current_scan_line_pub.publish(current_scan_line)
            self.stitched_point_cloud.header.stamp = stamp
            self.stitched_point_cloud_pub.publish(self.stitched_point_cloud)
        return


if __name__ == "__main__":
    p = PointCloudManager()
    rospy.spin()