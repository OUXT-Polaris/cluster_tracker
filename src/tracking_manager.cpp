#include <cluster_tracker/tracking_manager.h>

namespace cluster_tracker
{
    TrackingManager::TrackingManager(int num_tracking_threads)
    {

    }

    TrackingManager::~TrackingManager()
    {

    }

    void TrackingManager::addClusterClouds(std::vector<pcl::PCLPointCloud2> cluster_clouds,std::vector<vision_msgs::Detection3D> detections)
    {
        std::vector<pcl::PointCloud<RefPointType> > clouds;
        for(auto cluster_clouds_itr = cluster_clouds.begin(); cluster_clouds_itr != cluster_clouds.end(); cluster_clouds_itr++)
        {
            pcl::PointCloud<RefPointType>::Ptr temp_cloud(new pcl::PointCloud<RefPointType>);
            pcl::fromPCLPointCloud2(*cluster_clouds_itr,*temp_cloud);
            clouds.push_back(*temp_cloud);
        }
        addClusterClouds(clouds,detections);
    }

    void TrackingManager::addClusterClouds(std::vector<pcl::PointCloud<RefPointType> > cluster_clouds,std::vector<vision_msgs::Detection3D> detections)
    {

    }
}