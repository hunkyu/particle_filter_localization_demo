#include <iostream>
#include "matplotlib_eigen.h"
#include <vector>
#include "std_vector_to_edge_matrix.h"
#include "intersect_detection.h"
#include "sim_lidar_detection.h"
#include "footprint.h"
#include "mvn_rng.h"
#include "particle_filter.h"

using namespace std;
namespace plt = matplotlibcpp;

inline Eigen::Vector3d Transform2D(const Eigen::Vector3d& state_vector_in, const Eigen::Vector3d& transform)
{
    Eigen::Vector3d state_vector(state_vector_in);
    state_vector.block<2,1>(0,0) += 
        Eigen::Rotation2Dd(state_vector(2)).matrix() * transform.block<2,1>(0,0);
    state_vector(2) += transform(2);
    return state_vector;
}

int main(int argc, char** argv)
{
    vector<vector<double>> edge_map_0({
        {0,55,55,0,0},
        {0,0,55,55,0},
        {30,40,40,30},
        {5,20,15,5},
        {10,10,20,20,10},
        {24,38,38,24,24}
        });
    
    auto p_mat = ToEdgeMatrix(edge_map_0);
    auto & edge_mat = *p_mat;

    Footprint vehicle({-2.5, -2.5, 0, 2.5, 2.5}, {-2.5, 5, 7.5, 5, -2.5});
    Footprint veh_est(vehicle);
    veh_est.SetState((Eigen::Vector3d() << 35,20,1.5).finished());

    Eigen::Vector3d lidar_pose;
    lidar_pose << 38,25,0;
    vehicle.SetState(lidar_pose);
    
    int lidar_resolution = 16;
    double lidar_range = 50.;
    double lidar_measurement_variance = 0.1;
    double t_step = 0.05;
    double t = 0;
    Eigen::Vector3d v;
    v << 20, 0, 2;

    int particle_num = 32;
    Eigen::MatrixXd particles(particle_num, 3);
    Eigen::VectorXd init_weight(Eigen::MatrixXd::Constant(particle_num, 1, 1./particle_num));
    Eigen::MatrixXd init_covariance(Eigen::MatrixXd::Identity(3,3)*1.);
    Eigen::MatrixXd resample_covariance(Eigen::MatrixXd::Identity(3,3)*1e-2);

    Eigen::MatrixXd covariance(init_covariance);
    Eigen::VectorXd weight(init_weight);
    
    MVN_RNG rng(veh_est.GetState().transpose(), init_covariance);
    particles = *rng.GenerateRandoms(particle_num);
    while(true)
    {
        t += t_step;
        Eigen::Vector3d tf(v*t_step);
        vehicle.Transform(tf);
        lidar_pose = vehicle.GetState();
        Eigen::VectorXd measurement = 
            SimLidarDetection(lidar_pose, lidar_resolution, lidar_range, edge_mat);
        double best_likelihood = 0;
        {
            Eigen::MatrixXd measurement_particles(particle_num, lidar_resolution);
            for(int i = 0; i < particle_num; i++)
            {
                particles.row(i) = Transform2D(particles.row(i), tf);

                measurement_particles.row(i) = 
                    SimLidarDetection(particles.row(i), lidar_resolution, lidar_range, edge_mat);
            }
            
            auto p_weight = CalculateMeasurementWeight(
                measurement_particles, 
                measurement, 
                weight, 
                lidar_measurement_variance,
                best_likelihood);
            cout << "best_likelihood: " << best_likelihood << endl;
 
            weight = *p_weight;
            Eigen::Vector3d estimate = particles.transpose() * weight;
            covariance = *CalculateCovariance(particles, weight);
            veh_est.SetState(estimate);
            // cout << "weight: " << weight.transpose() << endl;
            cout << "est: " << veh_est.GetState().transpose() << endl;
        
            double shrink_coeff = best_likelihood > 0. ? 1e-3 : fabs(best_likelihood/(lidar_resolution*log(1e-6)));
            cout << "shrink_coeff: " << shrink_coeff << endl;
            
            Resampling(
                particles, 
                weight, 
                shrink_coeff * init_covariance,
                particle_num/2.);
        }
            
        auto VizMat = LidarMeasurementToVizPoint(measurement, veh_est.GetState(), lidar_resolution, lidar_range);

        plt::clf();
        plt::plot(VizMat,"r-");
        plt::plot(particles.block(0,0,particle_num, 2).matrix(), "g.");
        plt::plot(edge_map_0,"k");
        plt::plot(vehicle.GetVizVector(),"k");
        
        
        plt::plot(veh_est.GetVizVector(),"b--");
        plt::axis("equal");
        plt::xlim(-5,60);
        plt::ylim(-5,60);
        plt::pause(0.01);
        if(argc > 1)
            cin.get();
    }

    

    
    
}

//end

