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

    void TrackingManager::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
        return;
    }

    void TrackingManager::addNewDetections(std::vector<vision_msgs::Detection3D> detections)
    {
        return;
    }

    void TrackingManager::assignTracker(std::vector<vision_msgs::Detection3D> detections)
    {
        return;
    }
}