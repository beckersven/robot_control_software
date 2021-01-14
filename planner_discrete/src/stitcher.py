import rospy
from sensor_msgs.msg import PointCloud

class PointCloudStitcher:
    def __init__(self):
        rospy.init_node("point_cloud_stitcher")
        rospy.Subscriber("/measured_pointcloud", PointCloud, self.stitch)
        self.stitched_point_cloud = PointCloud()
        self.stitched_point_cloud_pub = rospy.Publisher("/stitched_pointcloud", PointCloud, queue_size=1, latch=True)

    def stitch(self, new_point_cloud):
        if self.stitched_point_cloud.header.frame_id == "":
            self.stitched_point_cloud.header.frame_id = new_point_cloud.header.frame_id
        if self.stitched_point_cloud.header.frame_id != new_point_cloud.header.frame_id:
            raise rospy.ROSException("Point-Cloud changed frame_id")
        
        self.stitched_point_cloud.points.extend(new_point_cloud.points)
        self.stitched_point_cloud_pub.publish(self.stitched_point_cloud)


if __name__ == "__main__":
    p = PointCloudStitcher()
    rospy.spin()