#ifndef CLUSTER_TRACKER_CLUSTER_TRACKER_H_INCLUDED
#define CLUSTER_TRACKER_CLUSTER_TRACKER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <quaternion_operation/quaternion_operation.h>
#include <jsk_rviz_plugins/OverlayText.h>

// Headers in this package
#include <cluster_tracker/ClusterTrackerConfig.h>
#include <cluster_tracker/tracking_manager.h>

// Headers in PCL
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// Headers in this package
#include <cluster_tracker/tracker_instance.h>

namespace cluster_tracker
{
    typedef message_filters::sync_policies::ApproximateTime<vision_msgs::Detection3DArray,sensor_msgs::PointCloud2> SyncPolicy;

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
        void clusterPointCloudCallback(const vision_msgs::Detection3DArray::ConstPtr& cluster,const sensor_msgs::PointCloud2::ConstPtr& cloud);
        std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection3DArray> > cluster_bbox_sub_ptr_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > pointcloud_sub_ptr_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_ptr_;
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