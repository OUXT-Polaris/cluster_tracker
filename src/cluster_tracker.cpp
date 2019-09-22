// Headers in this package
#include <cluster_tracker/cluster_tracker.h>

namespace cluster_tracker
{
    void ClusterTracker::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
        pnh_.param<int>("num_tracking_threads", num_tracking_threads_, 8);
        manager_ptr_ = std::make_shared<cluster_tracker::TrackingManager>(num_tracking_threads_);
        tracking_status_pub_ = pnh_.advertise<jsk_rviz_plugins::OverlayText>("tracking_status",1);
        param_func_ = boost::bind(&ClusterTracker::paramsCallback, this, _1, _2);
        param_server_.setCallback(param_func_);
        cluster_sub_ = pnh_.subscribe("input/cluster_bbox",1,&ClusterTracker::clusterCallback,this);
        twist_sub_ = pnh_.subscribe("input/current_twist",1,&ClusterTracker::twistStampedCallback,this);
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

    void ClusterTracker::paramsCallback(cluster_tracker::ClusterTrackerConfig &config, uint32_t level)
    {
        config_ = config;
        manager_ptr_->updateConfig(config_);
        return;
    }

    void ClusterTracker::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& twist)
    {
        return;
    }

    void ClusterTracker::clusterCallback(const vision_msgs::Detection3DArray::ConstPtr& cluster)
    {
        return;
    }

    jsk_rviz_plugins::OverlayText ClusterTracker::generateStatusText()
    {
        jsk_rviz_plugins::OverlayText status_text;
        status_text.action = jsk_rviz_plugins::OverlayText::ADD;
        status_text.width = 320;
        status_text.height = 320;
        status_text.left = 0;
        status_text.top = 0;
        status_text.bg_color.r = 0.0;
        status_text.bg_color.g = 0.0;
        status_text.bg_color.b = 0.0;
        status_text.bg_color.a = 0.8;
        status_text.line_width = 10;
        status_text.text_size = 10;
        status_text.fg_color.r = 0.2;
        status_text.fg_color.g = 0.2;
        status_text.fg_color.b = 0.8;
        status_text.fg_color.a = 0.8;
        status_text.text = "Tracking Status\n Objects:" + std::to_string(manager_ptr_->getNumberOfTrackingObjects());
        return status_text;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cluster_tracker::ClusterTracker,nodelet::Nodelet);