#ifndef _GRADIENTPATHTRACEBACK_H
#define _GRADIENTPATHTRACEBACK_H

#include <global_planner/planner_core.h>
#include <path_traceback/PathTraceback.h>
#include <algorithm>
#include <stdio.h>
#include <math.h>
// #include <cmath>
#include <utility>

#define MAX_COST 1.0e10 // the cost assigned to unassigned cells/obstacles

namespace global_planner {

class GradientPathTraceback : public PathTraceback {
    public:
        GradientPathTraceback();
        // GradientPathTraceback(const int nx, const int ny, const int ns): PathTraceback(nx, ny, ns), pathStep_(0.5) 
        // {
        //     gradx_ = grady_ = NULL;
        // }
        ~GradientPathTraceback();

        void setSize(const int nx, const int ny);
        
        // Path construction
        // Find gradient at array points, interpolate path
        // Use step size of pathStep, usually 0.5 pixel
        //
        // Some sanity checks:
        //  1. Stuck at same index position
        //  2. Doesn't get near goal
        //  3. Surrounded by high potentials
        //
        bool getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, const int gx, const int gy, std::vector<std::pair<float, float> >& path);

    private:
        float gradCell(std::vector<double> &costs, int n);

        float *gradx_, *grady_; /**< gradient arrays, size of potential array */

        float pathStep_; /**< step size for following gradient */

        // inline int getNearestPoint(int stc, float dx, float dy) {
        //     int pt = stc + (int)round(dx) + (int)(xs_ * round(dy));
        //     return std::max(0, std::min(xs_ * ys_ - 1, pt));
        // }


};

} //end namespace global_planner
#endif
