#ifndef SIM_LIDAR_DETECTION_H
#define SIM_LIDAR_DETECTION_H

#include "std_vector_to_edge_matrix.h"
#include "intersect_detection.h"

inline Eigen::VectorXd
SimLidarDetection(
    const Eigen::Vector3d& lidar_pose, 
    int resolution, 
    double lidar_range,
    const Eigen::MatrixXd& edge_map)
{
    double lidar_step = M_PI * 2 / resolution;
    double rotor_angle = 0.;
    Eigen::Vector3d rotor_pose(lidar_pose);
    Eigen::VectorXd dists(resolution);

    for(int step_count = 0; step_count < resolution; step_count++, rotor_angle += lidar_step)
    {
        rotor_pose(2) = lidar_pose(2) + rotor_angle;
        double min_dist = lidar_range;
        bool has_intersect = false;
        for(int i = 0; i < edge_map.rows(); i++)
        {
            if(true == IsIntersect2D(edge_map.row(i), rotor_pose))
            {
                auto intersect_point = IntersectPoint2D(edge_map.row(i), rotor_pose);
                double dist = (rotor_pose.block<2,1>(0,0) - intersect_point).norm();
                if(dist >= min_dist) continue;

                min_dist = dist;
                has_intersect = true;
            }
        }
        
        dists(step_count) = (has_intersect ? min_dist : 1e3);
    }
    return dists;
}

inline Eigen::MatrixXd
LidarMeasurementToVizPoint(
    const Eigen::VectorXd& measure, 
    const Eigen::Vector3d& lidar_pose,
    int resolution,
    double range)
{
    double lidar_step = M_PI * 2 / resolution;
    Eigen::MatrixXd VizMat( Eigen::MatrixXd::Zero(2, measure.size() * 2));
    for(int i = 0; i < measure.size(); i++)
    {
        if(std::isnan(measure(i))) continue;
        if(measure(i) >= range) continue;

        auto rotor_yaw = lidar_pose(2) + lidar_step * i;
        VizMat(0, 2*i) = lidar_pose(0);
        VizMat(0, 2*i+1) = lidar_pose(1);
        VizMat(1, 2*i) = lidar_pose(0) + measure(i)*cos(rotor_yaw);
        VizMat(1, 2*i+1) = lidar_pose(1) + measure(i)*sin(rotor_yaw);;
    }
    return VizMat;
}
#endif