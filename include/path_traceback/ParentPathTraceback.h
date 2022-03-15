#ifndef _PARENTPATHTRACEBACK_H
#define _PARENTPATHTRACEBACK_H

#include <path_traceback/PathTraceback.h>
#include <utility>
#include <vector>

namespace global_planner {

class ParentPathTraceback : public PathTraceback {
    public:
        ParentPathTraceback() {}
        // ParentPathTraceback(const int nx, const int ny, const int ns): PathTraceback(nx, ny, ns) {}
        bool getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, 
            const int gx, const int gy, std::vector<std::pair<float, float> >& path);
};

} //end namespace global_planner
#endif
