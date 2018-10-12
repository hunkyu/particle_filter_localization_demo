#ifndef MVN_PDF_H
#define MVN_PDF_H
//MVN: multi-variable normal distribution
//PDF: probability distribution function
#include <math.h>
#include <vector>
#include <utility>
#include <Eigen/Dense>

//multi dimension single modal
inline double gaussian(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& mean, 
    const Eigen::MatrixXd& cov_in)
{
    Eigen::MatrixXd cov = cov_in + Eigen::MatrixXd::Identity(cov_in.rows(), cov_in.cols()) * 1e-10;
    if(x.size() != mean.size() || x.size() != cov.cols())
    {
        std::string err_str("gaussian param size mismatch!");
        err_str += " x.size(): " + std::to_string(x.size());
        err_str += " mean.size(): " + std::to_string(mean.size());
        err_str += " cov.cols(): " + std::to_string(cov.cols());
        throw std::runtime_error(err_str);
    } 

    double coeff = 1/(sqrt(pow(2*M_PI, x.size()) * cov.determinant()));
    Eigen::VectorXd err_vec = x - mean;
    return coeff*exp(-0.5 * err_vec.transpose() * cov.inverse() * err_vec);
}

//multi dimension multi modal
inline double gaussian(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& mean, 
    const std::vector<Eigen::MatrixXd>& covs,
    const Eigen::VectorXd& weight)
{
    if(x.size() != mean.cols() || mean.rows() != covs.size() || weight.size() != mean.rows())
    {
        std::string err_str("gaussian param size mismatch!");
        err_str += " x.size(): " + std::to_string(x.size());
        err_str += " mean.cols(): " + std::to_string(mean.cols());
        err_str += " mean.rows(): " + std::to_string(mean.rows());
        err_str += " covs.size(): " + std::to_string(covs.size());
        err_str += " weight.size(): " + std::to_string(weight.size());
        throw std::runtime_error(err_str);
    } 
    if(fabs(weight.sum() - 1) > 1e-8)
    {
        std::string err_str("weight sum != 0!");
        err_str += " weight.sum(): " + std::to_string(weight.sum());
        throw std::runtime_error(err_str);
    }
    double ret = 0.0;
    for(int i = 0; i < mean.rows(); i++)
    {
        ret += gaussian(x, mean.row(i), covs.at(i)) * weight(i); 
    }
    return ret;
}

//multi dimension multi modal
inline double gaussian(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& mean, 
    const std::vector<Eigen::MatrixXd>& covs)
{
    Eigen::VectorXd weight(mean.rows());
    weight.array() = 1.0/mean.rows();
    double ret = gaussian(x, mean, covs, weight);

    return ret;
}

//single dimension single modal
inline double gaussian(
    const double x, 
    const double mean, 
    const double variance)
{
    Eigen::VectorXd x_in(1);
    Eigen::VectorXd mean_in(1);
    Eigen::MatrixXd cov_in(1,1);
    x_in << x;
    mean_in << mean;
    cov_in << variance;

    gaussian(x_in, mean_in, cov_in);
}

//single dimension multi modal
inline double gaussian(
    const double x, 
    const Eigen::VectorXd& mean, 
    const Eigen::VectorXd& variances)
{
    if(mean.size() != variances.size()) throw std::runtime_error("gaussian param size mismatch!");
    double ret = 1.0;
    for(int i = 0; i < mean.size(); i++)
    {
        ret *= gaussian(x, mean(i), variances(i)); 
    }
    return ret;
}

#endif