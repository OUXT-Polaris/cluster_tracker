// Headers in this package
#include <cluster_tracker/cluster_tracker.h>

namespace cluster_tracker
{
    void ClusterTracker::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
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
        return;
    }

    void ClusterTracker::clusterPointCloudCallback(const vision_msgs::Detection3DArray::ConstPtr& cluster,const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        sensor_msgs::PointCloud2 cloud_transformed;
        pcl::PCLPointCloud2::Ptr pcl_cloud_ptr(new pcl::PCLPointCloud2);
        if(cloud->header.frame_id != robot_frame_)
        {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(robot_frame_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.1));
            Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
            pcl_ros::transformPointCloud(mat, *cloud, cloud_transformed);
        }
        else
        {
            cloud_transformed = *cloud;
        }
        pcl_conversions::toPCL(cloud_transformed,*(pcl_cloud_ptr));
        vision_msgs::Detection3DArray cluster_transformed;
        cluster_transformed.header.frame_id = robot_frame_;
        cluster_transformed.header.stamp = cluster->header.stamp;
        for(auto itr=cluster->detections.begin(); itr!=cluster->detections.end(); itr++)
        {
            vision_msgs::Detection3D detection = *itr;
            detection.header.stamp = itr->header.stamp;
            detection.header.frame_id = robot_frame_;
            if(itr->header.frame_id!=robot_frame_)
            {
                geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(robot_frame_, itr->header.frame_id, itr->header.stamp, ros::Duration(0.1));
                geometry_msgs::PoseStamped bbox_pose;
                bbox_pose.header = itr->header;
                bbox_pose.pose = itr->bbox.center;
                tf2::doTransform(bbox_pose,bbox_pose,transform_stamped);
                detection.bbox.center = bbox_pose.pose;
            }
            else
            {
                detection.bbox = itr->bbox;
            }
        }
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cluster_tracker::ClusterTracker,nodelet::Nodelet);