#include <cluster_tracker/tracking_manager.h>

namespace cluster_tracker
{
    TrackingManager::TrackingManager(int num_tracking_threads)
    {
        num_tracking_threads_ = num_tracking_threads;
    }

    TrackingManager::~TrackingManager()
    {

    }

    void TrackingManager::trackObject()
    {

    }

    void TrackingManager::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
        return;
    }

    void TrackingManager::addNewDetections(std::vector<pcl::PCLPointCloud2> cluster_clouds,std::vector<vision_msgs::Detection3D> detections,pcl::PCLPointCloud2::Ptr whole_input_cloud)
    {
        std::vector<pcl::PointCloud<RefPointType> > clouds;
        for(auto cluster_clouds_itr = cluster_clouds.begin(); cluster_clouds_itr != cluster_clouds.end(); cluster_clouds_itr++)
        {
            pcl::PointCloud<RefPointType>::Ptr temp_cloud(new pcl::PointCloud<RefPointType>);
            pcl::fromPCLPointCloud2(*cluster_clouds_itr,*temp_cloud);
            clouds.push_back(*temp_cloud);
        }
        pcl::PointCloud<RefPointType>::Ptr tmp_whole_input_cloud(new pcl::PointCloud<RefPointType>);
        pcl::fromPCLPointCloud2(*whole_input_cloud,*tmp_whole_input_cloud);
        assignTracker(clouds,detections);
        return;
    }

    void TrackingManager::assignTracker(std::vector<pcl::PointCloud<RefPointType> > cluster_clouds,std::vector<vision_msgs::Detection3D> detections)
    {
        ROS_ASSERT(cluster_clouds.size() == detections.size());
        if(tracker_ptrs_.size() == 0)
        {
            for(int i=0; i<cluster_clouds.size(); i++)
            {
                std::shared_ptr<cluster_tracker::TrackerInstance> tracker_ptr = 
                    std::make_shared<cluster_tracker::TrackerInstance>(num_tracking_threads_,cluster_clouds[i],detections[i]);
                tracker_ptrs_.push_back(tracker_ptr);
            }
        }
        return;
    }
}