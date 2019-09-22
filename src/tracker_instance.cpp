// Headers in this package
#include <cluster_tracker/tracker_instance.h>

namespace cluster_tracker
{
    TrackerInstance::TrackerInstance(int num_tracking_threads, vision_msgs::Detection3D detection)
    {
    }

    TrackerInstance::~TrackerInstance()
    {

    }

    void TrackerInstance::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
        return;
    }

    boost::optional<double> TrackerInstance::getBboxMatchingCost(vision_msgs::Detection3D detection)
    {
        if(tracking_results_.size()==0)
        {
            return boost::none;
        }
        double ret = 0;
        return ret;
    }

    double TrackerInstance::getBboxMatchingCost(vision_msgs::BoundingBox3D bbox0, vision_msgs::BoundingBox3D bbox1)
    {
        double dist = std::sqrt(std::pow(bbox0.center.position.x-bbox1.center.position.x,2)
            +std::pow(bbox0.center.position.y-bbox1.center.position.y,2)
            +std::pow(bbox0.center.position.z-bbox1.center.position.z,2));
        double size = std::fabs(bbox0.size.x-bbox1.size.x) + std::fabs(bbox0.size.y-bbox1.size.y) + std::fabs(bbox0.size.z-bbox1.size.z);
        double ret = dist + size;
        return ret;
    }
}