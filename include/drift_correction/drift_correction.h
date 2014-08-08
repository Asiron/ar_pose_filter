#ifndef DRIFT_CORRECTION_HEADER_GUARD
#define DRIFT_CORRECTION_HEADER_GUARD

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <ar_pose_msgs/ARMarkers.h>
#include <ar_pose_msgs/ARMarker.h>

class DriftCorrector
{
public:
    DriftCorrector();
    ~DriftCorrector();

    void markerReceivedCallback(const ar_pose_msgs::ARMarkersConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle privateNh_;

    ros::Subscriber ar_pose_sub_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;


};

#endif
