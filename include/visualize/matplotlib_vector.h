#ifndef MATPLOTLIB_VECTOR_H
#define MATPLOTLIB_VECTOR_H

#include <vector>
#include <string>
#include "matplotlibcpp.h"

namespace matplotlibcpp{

template <typename T>
bool plot(const std::vector<std::vector<T>>& points, const std::string& format = "")
{
    for(int i = 0; i < points.size(); i+=2)
    {
        plot(points.at(i), points.at(i+1), format);
    }
}
}//namespace matplotlibcpp
#endif