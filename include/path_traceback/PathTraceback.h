#ifndef _PATHTRACEBACK_H
#define _PATHTRACEBACK_H

#include <ros/ros.h>
#include <vector>
#include <utility>

namespace global_planner {

class PathTraceback {
    public:
        PathTraceback(){}
        // PathTraceback(const int nx, const int ny, const int ns): nx_(nx), ny_(ny), ns_(ns) {}

        virtual void setSize(const int nx, const int ny)
        {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        }

        virtual bool getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, 
            const int gx, const int gy, std::vector<std::pair<float, float> >& path)=0;

    protected:
        void indexToMap(const int &idx, float &mx, float &my)
        {
            my = idx/nx_;
            mx = idx - my * nx_;
        }

        int mapToIndex(const float &mx, const float &my)
        {
            return mx + (my * nx_);
        }

        int nx_, ny_, ns_;

};

} //end namespace global_planner

#endif