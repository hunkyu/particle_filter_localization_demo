#ifndef MEASUREMENT_WEIGHT_H
#define MEASUREMENT_WEIGHT_H
#include <memory>
#include <Eigen/Dense>
#include "mvn_pdf.h"
#include "mvn_rng.h"
#include <vector>
#include <time.h>

inline void Resampling(
    Eigen::MatrixXd& particle_mat, 
    Eigen::VectorXd& weight_vec, 
    const Eigen::MatrixXd& estimate_covariance, 
    double n_th)
{
    double n_eff = 1./(weight_vec.transpose()*weight_vec)(0);
    if(n_eff > n_th) return;

    srand (time(NULL));
    std::cout << "do resample!" << std::endl;
    
    int particle_num = weight_vec.size();

    Eigen::VectorXd weight_cum_sum(weight_vec);
    Eigen::VectorXd base_cum_sum(Eigen::VectorXd::Constant(particle_num, 1, 1./particle_num));
    Eigen::VectorXd resample_id(particle_num);
    resample_id(0) = ((float)rand()/RAND_MAX)/particle_num;
    for(int i = 1; i < particle_num; i++)
    {
        weight_cum_sum(i) += weight_cum_sum(i-1);
        base_cum_sum(i) += base_cum_sum(i-1);
        resample_id(i) = base_cum_sum(i) + ((float)rand()/RAND_MAX - 1.)/particle_num;
    }
    base_cum_sum.array() -= 1./particle_num;
    
    std::vector<int> indices;
    int weight_idx = 0;
    for(int particle_idx = 0; particle_idx < particle_num; particle_idx++)
    {
        while(resample_id(particle_idx) > weight_cum_sum(weight_idx))
        {
            weight_idx++;
        }
            
        indices.push_back(weight_idx);
    }

    if(particle_mat.rows() != indices.size())
        throw std::runtime_error("dimension mismatch");

    for(int i = 0; i < particle_mat.rows(); i++)
    {
        particle_mat.row(i) = particle_mat.row(indices.at(i));
    }
    MVN_RNG rng(Eigen::VectorXd::Zero(particle_mat.cols()), estimate_covariance);
    particle_mat += *rng.GenerateRandoms(particle_num);
    weight_vec.array() = 1./particle_num;
}

inline double LogLikelihood(
    const Eigen::VectorXd& predict_vec, 
    const Eigen::VectorXd& observe_vec,
    const double& variance)
{
    const double min_likelihood = 1e-6;
    if(predict_vec.size() != observe_vec.size())
        throw std::runtime_error("vector dimension mismatch");
    int vec_size = predict_vec.size();
    double log_likelihood_sum = 0.;
    for(int i = 0; i < vec_size; i++)
    {
        double gauss = gaussian(predict_vec(i), observe_vec(i), variance);
        log_likelihood_sum += log(std::max(gauss, min_likelihood));
    }
    return log_likelihood_sum;
}

inline std::shared_ptr<Eigen::VectorXd>
CalculateMeasurementWeight(
    const Eigen::MatrixXd& predict_measurements, 
    const Eigen::VectorXd& mean_vector, 
    const Eigen::VectorXd& weight_vector, 
    double variance,
    double& log_best_likelihood)
{
    int particle_num = predict_measurements.rows();
    int measure_num = predict_measurements.cols();
    auto p_weight_vec = std::make_shared<Eigen::VectorXd>(particle_num);
    double norm_mean_vec = mean_vector.norm();
    Eigen::VectorXd log_sum_vec(particle_num);
    for(int i = 0; i < particle_num; i++)
    {
        log_sum_vec(i) = LogLikelihood(predict_measurements.row(i), mean_vector, variance);
    }
    std::cout << "log_sum_vec: " << log_sum_vec.transpose() << std::endl;
    log_best_likelihood = log_sum_vec.maxCoeff();
    log_sum_vec.array() -= log_sum_vec.maxCoeff();
    (*p_weight_vec) = (weight_vector.array() * log_sum_vec.array().exp()).matrix();

    double weight_sum = p_weight_vec->sum();
    
    (*p_weight_vec) /= weight_sum;
    return p_weight_vec;
}
#if 0
inline std::shared_ptr<Eigen::MatrixXd>
CalculateCovariance(
    const Eigen::MatrixXd& sample_mat, 
    const Eigen::VectorXd& weight_vec)
{
    int sample_num = sample_mat.rows();
    int dim = sample_mat.cols();
    if(sample_num != weight_vec.size())
        throw std::runtime_error("estimation and weight dimension mismatch!");
    
    auto p_covariance = std::make_shared<Eigen::MatrixXd>();
    Eigen::VectorXd sample_mean_vec(sample_mat.colwise().sum() / sample_num);
    Eigen::MatrixXd err_mat = sample_mat.rowwise() - sample_mean_vec.transpose();
    // std::cout << "err_mat: " << std::endl << err_mat << std::endl;
    Eigen::MatrixXd weight_diag(weight_vec.array().matrix().asDiagonal());
    // std::cout << "weight_diag: " << std::endl << weight_diag << std::endl;
    *p_covariance = err_mat.transpose() * weight_diag * err_mat;
    return p_covariance;
}
#else
inline std::shared_ptr<Eigen::MatrixXd>
CalculateCovariance(
    const Eigen::MatrixXd& sample_mat, 
    const Eigen::VectorXd& weight_vec)
{
    int sample_num = sample_mat.rows();
    int dim = sample_mat.cols();
    if(sample_num != weight_vec.size())
        throw std::runtime_error("estimation and weight dimension mismatch!");
    
    auto p_covariance = std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(dim, dim));
    Eigen::VectorXd sample_mean_vec(sample_mat.colwise().sum() / sample_num);
    Eigen::MatrixXd err_mat = sample_mat.rowwise() - sample_mean_vec.transpose();

    for(int i = 0; i < sample_num; i++)
        (*p_covariance) += weight_vec(i)*err_mat.transpose() *err_mat;

    return p_covariance;
}
#endif

inline Eigen::VectorXd ParticleFilterUpdate(
    const Eigen::VectorXd& actual_measure_vec, 
    const Eigen::MatrixXd& particle_measure_mat,
    Eigen::MatrixXd& particles,
    Eigen::VectorXd& weight_vec,
    double lidar_measurement_variance,
    const Eigen::MatrixXd& resample_covariance)
{
    int particle_num = particles.rows();
    int measurement_num = particle_measure_mat.rows();

    double best_likelihood = 0.;
    auto p_weight = CalculateMeasurementWeight(
        particle_measure_mat, 
        actual_measure_vec, 
        weight_vec, 
        lidar_measurement_variance,
        best_likelihood);
    std::cout << "best_likelihood: " << best_likelihood << std::endl;

    weight_vec = *p_weight;
    Eigen::VectorXd estimate = particles.transpose() * weight_vec;
    auto covariance = *CalculateCovariance(particles, weight_vec);

    std::cout << "est: " << estimate.transpose() << std::endl;

    double shrink_coeff = best_likelihood > 0. ? 1e-3 : fabs(best_likelihood/(measurement_num*log(1e-6)));
    std::cout << "shrink_coeff: " << shrink_coeff << std::endl;
    
    Resampling(
        particles, 
        weight_vec, 
        shrink_coeff * resample_covariance,
        particle_num/2.);
    return estimate;
}

#endif
