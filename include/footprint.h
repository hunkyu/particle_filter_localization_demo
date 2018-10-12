#include "matplotlibcpp.h"
#include <vector>
#include <Eigen/Dense>

class Footprint
{
public:
    Footprint(std::vector<double> x, std::vector<double> y)
    {
        if(x.size() != y.size())
        {
            throw std::runtime_error("Vertices x and y size mismatch!");
        }
        for(int i = 0; i < x.size(); i++)
        {
            vertices_.push_back(Eigen::Vector2d(x.at(i), y.at(i)));        
        }
        //to make close loop
        vertices_.push_back(Eigen::Vector2d(x.at(0), y.at(0)));        
    }

    std::vector<std::vector<double>> GetVizVector() const
    {
        std::vector<std::vector<double>> ret(2);

        Eigen::Rotation2Dd t(state_vector_(2) - M_PI/2);

        for(auto & v : vertices_)
        {
            for(int i = 0; i < 2; i++)
                ret[i].push_back((t.toRotationMatrix()*v)(i) + state_vector_(i));
        }
        return ret;
    }

    void Transform(Eigen::Vector3d tf)
    {
        // state_vector_ += tf;
        state_vector_.block<2,1>(0,0) += Eigen::Rotation2Dd(state_vector_(2)).matrix() * tf.block<2,1>(0,0);
        state_vector_(2) += tf(2);
    }

    void SetState(Eigen::Vector3d s)
    {
        state_vector_ = s;
    }

    Eigen::Vector3d GetState() const
    {
        return state_vector_;
    }

    Eigen::Vector3d& GetState()
    {
        return state_vector_;
    }

private:
    std::vector<Eigen::Vector2d> vertices_;
    Eigen::Vector3d state_vector_;// [x y theta]
};
