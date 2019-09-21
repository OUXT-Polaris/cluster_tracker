// Headers in this package
#include <cluster_tracker/tracker_instance.h>

namespace cluster_tracker
{
    TrackerInstance::TrackerInstance(int num_tracking_threads,pcl::PointCloud<RefPointType> cluster, vision_msgs::Detection3D detection)
    {
        tracking_results_ = boost::circular_buffer<vision_msgs::Detection3D>(10);
        // transform pointcloud
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        pcl::compute3DCentroid<RefPointType>(cluster, c);
        trans.translation().matrix() = Eigen::Vector3f (c[0],c[1],c[2]);
        model_ = pcl::PointCloud<RefPointType>::Ptr(new pcl::PointCloud<RefPointType>);
        pcl::transformPointCloud<RefPointType>(cluster,*model_,trans.inverse());

        // building model
        pcl::ApproximateVoxelGrid<RefPointType> grid;
        grid.setLeafSize(static_cast<float>(config_.downsample_grid_size),
            static_cast<float>(config_.downsample_grid_size),
            static_cast<float>(config_.downsample_grid_size));
        grid.setInputCloud(model_);
        grid.filter(*model_);

        // initialize tracker
        tracker_.setParticleNum(config_.maximum_particle_num);
        tracker_.setIterationNum(config_.inetration_num);
        tracker_.setTrans(Eigen::Affine3f::Identity());
        std::vector<double> default_step_covariance = std::vector<double> (6, config_.step_covariance_xyz);
        default_step_covariance[3] = config_.step_covariance_rpy;
        default_step_covariance[4] = config_.step_covariance_rpy;
        default_step_covariance[5] = config_.step_covariance_rpy;
        std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
        tracker_.setInitialNoiseCovariance(initial_noise_covariance);
        tracker_.setInitialNoiseMean(default_initial_mean);
        tracker_.setResampleLikelihoodThr(0.00);
        tracker_.setUseNormal(false);
        pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence(new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>);
        pcl::tracking::DistanceCoherence<RefPointType>::Ptr distance_coherence(new pcl::tracking::DistanceCoherence<RefPointType>);
        coherence->addPointCoherence(distance_coherence);
        pcl::search::Octree<RefPointType>::Ptr search(new pcl::search::Octree<RefPointType>(0.01));
        coherence->setSearchMethod(search);
        coherence->setMaximumDistance(0.01);
        tracker_.setCloudCoherence(coherence);
        tracker_.setReferenceCloud(model_);
        tracker_.setTrans(trans);
    }

    TrackerInstance::~TrackerInstance()
    {

    }

    void TrackerInstance::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
        return;
    }

    void TrackerInstance::trackObject(pcl::PCLPointCloud2::Ptr cloud)
    {
        return;
    }

    boost::optional<double> TrackerInstance::getBboxMatchingCost(pcl::PointCloud<RefPointType> cluster,vision_msgs::Detection3D detection)
    {
        double ret;
        return ret;
    }

    double TrackerInstance::getPointCloudMatchingCost(pcl::PointCloud<RefPointType> pc0, pcl::PointCloud<RefPointType> pc1)
    {
        pcl::SegmentDifferences<pcl::PointCloud<RefPointType> > impl_;
        impl_.setDistanceThreshold(config_.pointcloud_distance_threashold);
        //impl_.setInputCloud(pc0);
        //impl_.setTargetCloud(pc1);
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