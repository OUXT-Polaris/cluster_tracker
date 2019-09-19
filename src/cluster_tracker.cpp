// Headers in this package
#include <cluster_tracker/cluster_tracker.h>

namespace cluster_tracker
{
    void ClusterTracker::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        cluster_bbox_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection3DArray> >(pnh_, "input/cluster_bbox", 10);
        pointcloud_sub_ptr_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2> >(pnh_, "input/pointcloud", 10);
        param_func_ = boost::bind(&ClusterTracker::paramsCallback, this, _1, _2);
        param_server_.setCallback(param_func_);
        sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10),*cluster_bbox_sub_ptr_, *pointcloud_sub_ptr_);
        sync_ptr_->registerCallback(boost::bind(&ClusterTracker::clusterPointCloudCallback, this, _1, _2));
    }

    void ClusterTracker::paramsCallback(cluster_tracker::ClusterTrackerConfig &config, uint32_t level)
    {
        config_ = config;
        return;
    }

    void ClusterTracker::clusterPointCloudCallback(const vision_msgs::Detection3DArray::ConstPtr cluster,const sensor_msgs::PointCloud2::ConstPtr cloud)
    {
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cluster_tracker::ClusterTracker,nodelet::Nodelet);