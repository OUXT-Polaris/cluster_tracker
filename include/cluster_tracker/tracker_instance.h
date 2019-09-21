#ifndef CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED
#define CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED

// Headers in ROS
#include <vision_msgs/Detection3D.h>

// Headers in PCL
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <boost/circular_buffer.hpp>

// Headers in this package
#include <cluster_tracker/ClusterTrackerConfig.h>

namespace cluster_tracker
{
    typedef pcl::PointXYZ RefPointType;
    typedef pcl::tracking::ParticleXYZRPY ParticleT;
    typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
    
    class TrackerInstance
    {
    public:
        TrackerInstance(int num_tracking_threads,pcl::PointCloud<RefPointType> cluster, vision_msgs::Detection3D detection);
        ~TrackerInstance();
        void updateConfig(cluster_tracker::ClusterTrackerConfig config);
        void trackObject(pcl::PCLPointCloud2::Ptr cloud,pcl::PointCloud<RefPointType> cluster, vision_msgs::Detection3D detection);
    private:
        ParticleFilter tracker_;
        double getBboxMatchingCost(vision_msgs::BoundingBox3D bbox0, vision_msgs::BoundingBox3D bbox1);
        cluster_tracker::ClusterTrackerConfig config_;
        pcl::PointCloud<RefPointType>::Ptr model_;
        boost::circular_buffer<vision_msgs::Detection3D> tracking_results_;
    };
}

#endif  //CLUSTER_TRACKER_TRACKER_INSTANCE_H_INCLUDED