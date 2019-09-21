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

namespace cluster_tracker
{
    typedef pcl::PointXYZI RefPointType;
    typedef pcl::tracking::ParticleXYZRPY ParticleT;
    typedef pcl::tracking::ParticleFilterTracker<pcl::PCLPointCloud2, ParticleT> ParticleFilter;
    class TrackingManager
    {
    public:
        TrackingManager(int num_tracking_threads);
        ~TrackingManager();
        void addClusterClouds(std::vector<pcl::PCLPointCloud2> cluster_clouds,std::vector<vision_msgs::Detection3D> detections);
        void addClusterClouds(std::vector<pcl::PointCloud<RefPointType> > cluster_clouds,std::vector<vision_msgs::Detection3D> detections);
    private:
        std::vector<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr> tracker_ptrs_;
        void addNewDetection(pcl::PointCloud<RefPointType> detction_cloud,vision_msgs::Detection3D detection);
    };
}

#endif  //CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED