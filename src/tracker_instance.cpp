// Headers in this package
#include <cluster_tracker/tracker_instance.h>

namespace cluster_tracker
{
    TrackerInstance::TrackerInstance(int num_tracking_threads)
    {
        tracker_ptr_ = pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr
            (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (num_tracking_threads));
    }

    TrackerInstance::TrackerInstance()
    {
        tracker_ptr_ = pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr
            (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));
    }

    TrackerInstance::~TrackerInstance()
    {

    }

    void TrackerInstance::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
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