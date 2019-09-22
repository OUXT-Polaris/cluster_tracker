#ifndef CLUSTER_TRACKER_CLUSTER_TRACKER_H_INCLUDED
#define CLUSTER_TRACKER_CLUSTER_TRACKER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <quaternion_operation/quaternion_operation.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <geometry_msgs_data_buffer/twist_stamped_data_buffer.h>

// Headers in this package
#include <cluster_tracker/ClusterTrackerConfig.h>
#include <cluster_tracker/tracking_manager.h>

// Headers in this package
#include <cluster_tracker/tracker_instance.h>

namespace cluster_tracker
{
    class ClusterTracker: public nodelet::Nodelet
    {
    protected:
        void onInit();
    private:
        void paramsCallback(cluster_tracker::ClusterTrackerConfig &config, uint32_t level);
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        cluster_tracker::ClusterTrackerConfig config_;
        dynamic_reconfigure::Server<cluster_tracker::ClusterTrackerConfig> param_server_;
        dynamic_reconfigure::Server<cluster_tracker::ClusterTrackerConfig>::CallbackType param_func_;
        void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& twist);
        ros::Subscriber twist_sub_;
        void clusterCallback(const vision_msgs::Detection3DArray::ConstPtr& cluster);
        ros::Subscriber cluster_sub_;
        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
        std::string robot_frame_;
        int num_tracking_threads_;
        std::shared_ptr<cluster_tracker::TrackingManager> manager_ptr_;
        ros::Publisher tracking_status_pub_;
        jsk_rviz_plugins::OverlayText generateStatusText();
    };
}

#endif  //CLUSTER_TRACKER_CLUSTER_TRACKER_H_INCLUDED