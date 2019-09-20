#ifndef CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED
#define CLUSTER_TRACKER_TRACKING_MANAGER_H_INCLUDED

#include <hungarian_solver/hungarian_solver.h>

namespace cluster_tracker
{
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