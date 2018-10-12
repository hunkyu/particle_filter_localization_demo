#ifndef INTERSECT_DETECTION_H
#define INTERSECT_DETECTION_H
#include <Eigen/Dense>

inline Eigen::Vector2d 
IntersectPoint2D(const Eigen::Vector4d& line_segment, const Eigen::Vector3d& pose_2d)
{
    double x1 = line_segment(0);
    double y1 = line_segment(1);
    double x2 = line_segment(2);
    double y2 = line_segment(3);
    double x0 = pose_2d(0);
    double y0 = pose_2d(1);
    double yaw0 = pose_2d(2);

    Eigen::MatrixXd A(2,2);
    A << 
    -(y2 - y1) , (x2 - x1),
    -sin(yaw0) , cos(yaw0);

    Eigen::Vector2d b;
    b << 
    (y1*(x2 - x1) - x1*(y2 - y1)),
    (y0*cos(yaw0) - x0*sin(yaw0));

    return A.inverse()*b;
}

inline bool
IsIntersect2D(const Eigen::Vector4d& line_segment, const Eigen::Vector3d& pose_2d)
{
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;

    p0 << pose_2d.block<2,1>(0,0), 0;
    p1 << line_segment.block<2,1>(0,0), 0;
    p2 << line_segment.block<2,1>(2,0), 0;

    Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d v1 = p1 - p0;
    Eigen::Vector3d v0;
    v0 << cos(pose_2d(2)), sin(pose_2d(2)), 0;

    double cross10 = (v1.cross(v0))(2);
    double cross02 = (v0.cross(v2))(2);
    double cross12 = (v1.cross(v2))(2);

    return (cross10 * cross12 > 0 && cross10 * cross02 > 0);
}

#endif