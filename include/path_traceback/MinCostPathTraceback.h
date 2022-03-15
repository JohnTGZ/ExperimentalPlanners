#ifndef _MINCOSTPATHTRACEBACK_H
#define _MINCOSTPATHTRACEBACK_H

#include <path_traceback/PathTraceback.h>
#include <algorithm>
#include <vector>
#include <utility>
#include <stdio.h>

namespace global_planner {

class MinCostPathTraceback : public PathTraceback {
    public:
        MinCostPathTraceback() {}
        // MinCostPathTraceback(const int nx, const int ny, const int ns): 
        //     PathTraceback(nx, ny, ns) {}
        bool getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, 
            const int gx, const int gy, std::vector<std::pair<float, float> >& path);
};

} //end namespace global_planner
#endif
