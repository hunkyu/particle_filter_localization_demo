#ifndef MATPLOTLIB_EIGEN_H
#define MATPLOTLIB_EIGEN_H

#include <vector>
#include "matplotlib_vector.h"
#include <Eigen/Dense>

namespace matplotlibcpp{

template<typename T, int U, int V>
bool plot(const Eigen::Matrix<T, U, V>& points, const std::string& format = "")
{
    std::vector<std::vector<double>> viz_vec(points.cols());
    
    for(int i = 0; i < viz_vec.size(); i++)
    {
        viz_vec.at(i).resize(points.rows());
        Eigen::VectorXd::Map(&viz_vec.at(i)[0], viz_vec.at(i).size()) = points.col(i);
    }
    plot(viz_vec, format);
}

template<typename W>
bool plot(const Eigen::MatrixBase<W>& points, const std::string& format = "")
{
    std::vector<std::vector<double>> viz_vec(points.cols());
    
    for(int i = 0; i < viz_vec.size(); i++)
    {
        viz_vec.at(i).resize(points.rows());
        Eigen::VectorXd::Map(&viz_vec.at(i)[0], viz_vec.at(i).size()) = points.col(i);
    }
    plot(viz_vec, format);
}
}//namespace matplotlibcpp
#endif