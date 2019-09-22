#ifndef CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED
#define CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED

// Headers in ROS
#include <vision_msgs/Detection3D.h>

// Headers in PCL
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

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
    typedef pcl::PointXYZ RefPointType;

    class TrackingManager
    {
    public:
        TrackingManager(int num_tracking_threads);
        ~TrackingManager();
        void addNewDetections(std::vector<pcl::PCLPointCloud2> cluster_clouds,std::vector<vision_msgs::Detection3D> detections,pcl::PCLPointCloud2::Ptr whole_input_cloud);
        void updateConfig(cluster_tracker::ClusterTrackerConfig config);
        int getNumberOfTrackingObjects()
        {
            return (int)tracker_ptrs_.size();
        }
    private:
        void trackObject(pcl::PCLPointCloud2::Ptr whole_input_cloud);
        void assignTracker(std::vector<pcl::PointCloud<RefPointType> > cluster_clouds,std::vector<vision_msgs::Detection3D> detections);
        int num_tracking_threads_;
        std::vector<std::shared_ptr<cluster_tracker::TrackerInstance> > tracker_ptrs_;
        cluster_tracker::ClusterTrackerConfig config_;
    };
}

#endif  //CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED