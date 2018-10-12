#ifndef MVN_RNG_H
#define MVN_RNG_H
// MVN: multi-variable normal distribution
// RNG: random number generator

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <sstream>

class MVN_RNG
{
public:
    
    MVN_RNG(const Eigen::VectorXd& means, const Eigen::MatrixXd& cov)
        :e2_(rd_()),nd_(means.size()), means_(means), cov_(cov)
    {
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov_);
        cov_eigenvalues_ = es.eigenvalues().real();
        cov_eigenvectors_ = es.eigenvectors().real();
        for(int i = 0; i < nd_; i++)
        {
            if(cov_eigenvalues_(i) < 0) 
            {
                std::stringstream err_msg;
                err_msg <<"covariance matrix not positive semi-definite!";
                err_msg << std::endl << cov_eigenvalues_ << std::endl;
                throw std::runtime_error(err_msg.str());
            }
                
        }
    }
    const Eigen::VectorXd& GetMeans() {return means_;}
    const Eigen::MatrixXd& GetCovariance() {return cov_;}
    const Eigen::MatrixXd& GetRotateMatrix() {return cov_eigenvectors_;}
    const Eigen::VectorXd& GetCovEigenvalues() {return cov_eigenvalues_;}

    void GenerateRandoms(uint16_t num, Eigen::MatrixXd& points)
    {
        points.resize(num, nd_);

        for(int i = 0; i < num; i++)
        {
            for(int j = 0; j < nd_; j++)
            {
                std::normal_distribution<double> distribution(means_(j), sqrt(cov_eigenvalues_(j)));
                points(i, j) = distribution(e2_);
            }
            points.row(i) = points.row(i) - means_.transpose();
            points.row(i) = cov_eigenvectors_ * points.row(i).transpose();
            points.row(i) = points.row(i) + means_.transpose();
        }
    }

    std::shared_ptr<Eigen::MatrixXd> GenerateRandoms(uint16_t num)
    {
        std::shared_ptr<Eigen::MatrixXd> ret(std::make_shared<Eigen::MatrixXd>());
        GenerateRandoms(num, *ret);
        return ret;
    }

private:
    const uint16_t nd_;
    std::random_device rd_;
    std::mt19937 e2_;
    Eigen::VectorXd means_;
    Eigen::MatrixXd cov_;
    Eigen::VectorXd cov_eigenvalues_;
    Eigen::MatrixXd cov_eigenvectors_;
};
#endif