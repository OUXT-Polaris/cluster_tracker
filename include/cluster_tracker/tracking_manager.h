#ifndef CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED
#define CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED

// Headers in ROS
#include <hungarian_solver/hungarian_solver.h>

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

namespace cluster_tracker
{
    typedef pcl::tracking::ParticleXYZRPY ParticleT;
    typedef pcl::tracking::ParticleFilterTracker<pcl::PCLPointCloud2, ParticleT> ParticleFilter;

    class TrackingManager
    {
    public:
        TrackingManager();
        ~TrackingManager();
    private:
        hungarian_solver::Solver solver_;
    };
}

#endif  //CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED