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
        cluster_bbox_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection3DArray> >(pnh_, "input/cluster_bbox", 10);
        pointcloud_sub_ptr_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2> >(pnh_, "input/pointcloud", 10);
        param_func_ = boost::bind(&ClusterTracker::paramsCallback, this, _1, _2);
        param_server_.setCallback(param_func_);
        sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10),*cluster_bbox_sub_ptr_, *pointcloud_sub_ptr_);
        sync_ptr_->registerCallback(boost::bind(&ClusterTracker::clusterPointCloudCallback, this, _1, _2));
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

    void ClusterTracker::paramsCallback(cluster_tracker::ClusterTrackerConfig &config, uint32_t level)
    {
        config_ = config;
        manager_ptr_->updateConfig(config_);
        return;
    }

    void ClusterTracker::clusterPointCloudCallback(const vision_msgs::Detection3DArray::ConstPtr& cluster,const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        std::vector<pcl::PCLPointCloud2> cluster_clouds;
        std::vector<vision_msgs::Detection3D> detections;
        pcl::PCLPointCloud2::Ptr pcl_cloud_ptr(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*cloud,*(pcl_cloud_ptr));
        ROS_ASSERT(cloud->header.frame_id == robot_frame_);
        for(auto itr=cluster->detections.begin(); itr!=cluster->detections.end(); itr++)
        {
            ROS_ASSERT(itr->header.frame_id == robot_frame_);
        }
        ROS_ASSERT(cluster->header.frame_id == robot_frame_);
        for(auto itr=cluster->detections.begin(); itr!=cluster->detections.end(); itr++)
        {
            pcl::CropBox<pcl::PCLPointCloud2> crop_box;
            crop_box.setInputCloud(pcl_cloud_ptr);
            Eigen::Vector3f translation;
            translation << itr->bbox.center.position.x, itr->bbox.center.position.y, itr->bbox.center.position.z;
            crop_box.setTranslation(translation);
            Eigen::Vector3f rotation;
            geometry_msgs::Vector3 rot_vec = quaternion_operation::convertQuaternionToEulerAngle(itr->bbox.center.orientation);
            rotation << rot_vec.x, rot_vec.y, rot_vec.z;
            crop_box.setRotation(rotation);
            crop_box.filter(*pcl_cloud_ptr);
            cluster_clouds.push_back(*pcl_cloud_ptr);
            detections.push_back(*itr);
        }
        manager_ptr_->addNewDetections(cluster_clouds,detections,pcl_cloud_ptr);

        jsk_rviz_plugins::OverlayText status_text = generateStatusText();
        tracking_status_pub_.publish(status_text);

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