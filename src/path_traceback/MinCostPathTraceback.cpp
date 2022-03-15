#include <path_traceback/MinCostPathTraceback.h>

namespace global_planner {

bool MinCostPathTraceback::getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, 
            const int gx, const int gy, std::vector<std::pair<float, float> >& path)
{
    ROS_INFO("Extracting min-cost path");
    std::pair<float, float> current;
    current.first = gx;
    current.second = gy;

    path.push_back(current);
    int c = 0;

    int current_idx = mapToIndex(gx, gy);
    int start_idx = mapToIndex(sx, sy);

    while (mapToIndex(current.first, current.second) != start_idx) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = mapToIndex(x, y);
                if (costs[index] < min_val) {
                    min_val = costs[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)
            return false;
        current.first = min_x;
        current.second = min_y;
        path.push_back(current);
        
        if(c++>ns_*4){
            return false;
        }

    }
    return true;

}

} //end namespace global_planner

