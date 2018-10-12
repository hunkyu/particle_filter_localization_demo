#ifndef STD_VECTOR_TO_EDGE_MATRIX_H
#define STD_VECTOR_TO_EDGE_MATRIX_H
#include <Eigen/Dense>
#include <vector>
#include <memory>
std::shared_ptr<Eigen::MatrixXd> 
ToEdgeMatrix(const std::vector<std::vector<double>>& edgess)
{
    auto p_edge_matrix = std::make_shared<Eigen::MatrixXd>();
    int total_edge_num = 0;
    for(const auto& edges : edgess)
        total_edge_num += edges.size();

    std::vector<double> vec_x1;
    std::vector<double> vec_y1;
    std::vector<double> vec_x2;
    std::vector<double> vec_y2;
    for(int i = 0; i < edgess.size(); i+= 2)
    {
        if(edgess.at(i).size() != edgess.at(i+1).size())
            throw std::runtime_error("x,y dimension mismatch!");
        
        for(int j = 0; j < edgess.at(i).size() - 1; j++)
        {
            vec_x1.push_back(edgess.at(i).at(j));
            vec_y1.push_back(edgess.at(i+1).at(j));
            vec_x2.push_back(edgess.at(i).at(j+1));
            vec_y2.push_back(edgess.at(i+1).at(j+1));
        }
    }
    
    p_edge_matrix->resize(vec_x1.size(), 2*2);
    for(int i = 0; i < p_edge_matrix->rows(); i++)
    {
        (*p_edge_matrix)(i,0) = vec_x1.at(i);
        (*p_edge_matrix)(i,1) = vec_y1.at(i);
        (*p_edge_matrix)(i,2) = vec_x2.at(i);
        (*p_edge_matrix)(i,3) = vec_y2.at(i);
    }

    return p_edge_matrix;
}
#endif