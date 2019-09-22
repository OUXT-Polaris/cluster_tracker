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

    void TrackingManager::trackObject(pcl::PCLPointCloud2::Ptr whole_input_cloud)
    {
        pcl::PointCloud<RefPointType>::Ptr temp_cloud(new pcl::PointCloud<RefPointType>);
        pcl::fromPCLPointCloud2(*whole_input_cloud,*temp_cloud);
        for(auto tracker_itr=tracker_ptrs_.begin(); tracker_itr!=tracker_ptrs_.end(); tracker_itr++)
        {
            std::shared_ptr<cluster_tracker::TrackerInstance> tracker_ptr = *tracker_itr;
            tracker_ptr->trackObject(temp_cloud);
        }
        return;
    }

    void TrackingManager::updateConfig(cluster_tracker::ClusterTrackerConfig config)
    {
        config_ = config;
        return;
    }

    void TrackingManager::addNewDetections(std::vector<pcl::PCLPointCloud2> cluster_clouds,std::vector<vision_msgs::Detection3D> detections,pcl::PCLPointCloud2::Ptr whole_input_cloud)
    {
        trackObject(whole_input_cloud);
        std::vector<pcl::PointCloud<RefPointType> > clouds;
        for(auto cluster_clouds_itr = cluster_clouds.begin(); cluster_clouds_itr != cluster_clouds.end(); cluster_clouds_itr++)
        {
            pcl::PointCloud<RefPointType>::Ptr temp_cloud(new pcl::PointCloud<RefPointType>);
            pcl::fromPCLPointCloud2(*cluster_clouds_itr,*temp_cloud);
            clouds.push_back(*temp_cloud);
        }
        pcl::PointCloud<RefPointType>::Ptr tmp_whole_input_cloud(new pcl::PointCloud<RefPointType>);
        pcl::fromPCLPointCloud2(*whole_input_cloud,*tmp_whole_input_cloud);
        assignTracker(clouds,detections);
        return;
    }

    void TrackingManager::assignTracker(std::vector<pcl::PointCloud<RefPointType> > cluster_clouds,std::vector<vision_msgs::Detection3D> detections)
    {
        ROS_ASSERT(cluster_clouds.size() == detections.size());
        if(tracker_ptrs_.size() == 0)
        {
            for(int i=0; i<cluster_clouds.size(); i++)
            {
                std::shared_ptr<cluster_tracker::TrackerInstance> tracker_ptr = 
                    std::make_shared<cluster_tracker::TrackerInstance>(num_tracking_threads_,cluster_clouds[i],detections[i]);
                tracker_ptrs_.push_back(tracker_ptr);
            }
        }
        else
        {
            Eigen::MatrixXd cost_matrix(cluster_clouds.size(),tracker_ptrs_.size());
            for(int r=0; r<cluster_clouds.size(); r++)
            {
                for(int c=0; c<tracker_ptrs_.size(); c++)
                {
                    boost::optional<double> cost = tracker_ptrs_[c]->getBboxMatchingCost(cluster_clouds[r],detections[r]);
                    if(cost)
                    {
                        cost_matrix(r,c) = cost.get();
                    }
                    else
                    {
                        cost_matrix(r,c) = 100.0;
                    }
                }
            }
            try
            {
                boost::optional<std::vector<std::pair<int,int> > > match = solver_.solve(cost_matrix,100.0);
                if(!match)
                {
                    for(int i=0; i<cluster_clouds.size(); i++)
                    {
                        std::shared_ptr<cluster_tracker::TrackerInstance> tracker_ptr = 
                            std::make_shared<cluster_tracker::TrackerInstance>(num_tracking_threads_,cluster_clouds[i],detections[i]);
                        tracker_ptrs_.push_back(tracker_ptr);
                    }
                }
                else
                {
                    for(int r=0; r<cluster_clouds.size(); r++)
                    {
                        bool tracker_matched = false;
                        for(int c=0; c<tracker_ptrs_.size(); c++)
                        {
                            if(isMatched(r,c,*match))
                            {
                                if(cost_matrix(r,c)<config_.matching_cost_threashold)
                                {
                                    tracker_ptrs_[c]->updateModel(cluster_clouds[r]);
                                }
                                tracker_matched = true;
                            }
                        }
                        if(!tracker_matched)
                        {
                            std::shared_ptr<cluster_tracker::TrackerInstance> tracker_ptr = 
                                std::make_shared<cluster_tracker::TrackerInstance>(num_tracking_threads_,cluster_clouds[r],detections[r]);
                            tracker_ptrs_.push_back(tracker_ptr);
                        }
                    }
                }
            }
            catch(...)
            {
                ROS_WARN_STREAM("Failed to find match result.");
                return;
            }
        }
        return;
    }
}