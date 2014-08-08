#include <drift_correction/drift_correction.h>

DriftCorrector::DriftCorrector()
{
    ar_pose_sub_ = nh_.subscribe<ar_pose_msgs::ARMarkers>("ar_pose_marker", 1000, boost::bind(&DriftCorrector::markerReceivedCallback, this, _1));
}

DriftCorrector::~DriftCorrector()
{

}

void DriftCorrector::markerReceivedCallback(const ar_pose_msgs::ARMarkersConstPtr& msg)
{
    for (const auto marker : msg->markers) {
        ROS_INFO("marker:%s\nposition:%f %f %f\nquaternion:%f %f %f %f\nconfidence:%d",
                 marker.header.frame_id.c_str(),
                 marker.pose.pose.position.x,
                 marker.pose.pose.position.y,
                 marker.pose.pose.position.z,
                 marker.pose.pose.orientation.x,
                 marker.pose.pose.orientation.y,
                 marker.pose.pose.orientation.z,
                 marker.pose.pose.orientation.w,
                 marker.confidence);
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drift_corrector");
    DriftCorrector corrector;
    ros::spin();
    return 0;
}
