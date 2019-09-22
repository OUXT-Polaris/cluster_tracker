#ifndef CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED
#define CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED

// Headers in ROS
#include <vision_msgs/Detection3D.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

// Headers in this package
#include <cluster_tracker/ClusterTrackerConfig.h>

namespace cluster_tracker
{   
    class TrackerInstance
    {
    public:
        TrackerInstance(int num_tracking_threads,vision_msgs::Detection3D detection);
        ~TrackerInstance();
        void updateConfig(cluster_tracker::ClusterTrackerConfig config);
        boost::optional<double> getBboxMatchingCost(vision_msgs::Detection3D detection);
    private:
        double getBboxMatchingCost(vision_msgs::BoundingBox3D bbox0, vision_msgs::BoundingBox3D bbox1);
        cluster_tracker::ClusterTrackerConfig config_;
        boost::circular_buffer<vision_msgs::Detection3D> tracking_results_;
    };
}

#endif  //CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED