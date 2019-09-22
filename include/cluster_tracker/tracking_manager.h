#ifndef CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED
#define CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED

// Headers in ROS
#include <vision_msgs/Detection3D.h>
#include <hungarian_solver/hungarian_solver.h>

// Headers in Boost
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/assign/list_of.hpp>

// Headers in this package
#include <cluster_tracker/tracker_instance.h>
#include <cluster_tracker/ClusterTrackerConfig.h>

namespace cluster_tracker
{
    class TrackingManager
    {
    public:
        TrackingManager(int num_tracking_threads);
        ~TrackingManager();
        void addNewDetections(std::vector<vision_msgs::Detection3D> detections);
        void updateConfig(cluster_tracker::ClusterTrackerConfig config);
        int getNumberOfTrackingObjects()
        {
            return (int)tracker_ptrs_.size();
        }
    private:
        void assignTracker(std::vector<vision_msgs::Detection3D> detections);
        int num_tracking_threads_;
        std::vector<std::shared_ptr<cluster_tracker::TrackerInstance> > tracker_ptrs_;
        cluster_tracker::ClusterTrackerConfig config_;
        hungarian_solver::Solver solver_;
    };
}

#endif  //CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED