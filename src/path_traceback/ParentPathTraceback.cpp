#include <path_traceback/ParentPathTraceback.h>

namespace global_planner {

    bool ParentPathTraceback::getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, 
        const int sy, const int gx, const int gy, std::vector<std::pair<float, float> >& path)
    {
        ROS_INFO("Extract parent traceback path");
        std::vector<std::pair<float, float> > path_reversed;
        std::pair<float, float> current;
        current.first = gx;
        current.second = gy;

        path.push_back(current);

        int current_idx = mapToIndex(gx, gy);
        int start_idx = mapToIndex(sx, sy);
        
        while (parents[current_idx] != start_idx) 
        {
            indexToMap(current_idx, current.first, current.second);
            path.push_back(current); 
            current_idx = parents[current_idx]; //current_idx is pointing to parent cell
        }

        //add start cell to path
        current.first = sx;
        current.second = sy;
        path.push_back(current);

        // //reverse path to right order
        // int len_plan = path_reversed.size();
        // for (int i = len_plan-1; i>=0; --i)
        // {
        //     path.push_back(path_reversed[i]);
        // }
    }

} //end namespace global_planner


