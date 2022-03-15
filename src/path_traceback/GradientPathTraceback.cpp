#include <path_traceback/GradientPathTraceback.h>

namespace global_planner {

GradientPathTraceback::GradientPathTraceback() : pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

GradientPathTraceback::~GradientPathTraceback() 
{
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPathTraceback::setSize(const int nx, const int ny)
{
    PathTraceback::setSize(nx, ny);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[nx * ny];
    grady_ = new float[nx * ny];
}

bool GradientPathTraceback::getPath(std::vector<int> &parents, std::vector<double> &costs, const int sx, const int sy, const int gx, const int gy, std::vector<std::pair<float, float> >& path) 
{
    ROS_INFO("Extracting gradient path!");
    std::pair<float, float> current;
    int stc = mapToIndex(gx, gy); 

    // set up offset
    float dx = gx - (int)gx;
    float dy = gy - (int)gy;
    memset(gradx_, 0, ns_ * sizeof(float));
    memset(grady_, 0, ns_ * sizeof(float));

    int c = 0;
    while (c++<ns_*4) {
        double nx = stc % nx_ + dx, ny = stc / nx_ + dy; //get map coordinates of cell index stc

        // check if the cell index stc is near the start cell
        if (fabs(nx - sx) < .5 && fabs(ny - sy) < .5) {
            current.first = sx;
            current.second = sy;
            path.push_back(current);
            return true;
        }

        // check if cell index stc is out of bounds
        if (stc < nx_ || stc > nx_ * ny_ - nx_)
        {
            printf("[PathCalc] Out of bounds\n");
            return false;
        }

        current.first = nx;
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%nx_, stc/nx_, dx, dy);

        path.push_back(current); //push back goal cell first

        //check for any oscillation in path
        bool oscillation_detected = false;
        int npath = path.size(); 
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first
                && path[npath - 1].second == path[npath - 3].second) {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        int stcnx = stc + nx_; //set up values for checking in octile directions
        int stcpx = stc - nx_;

        // If any neighboring cell is obstacle or over inflated or oscillation is true,
        // then check for costs at eight positions near cell
        if (costs[stc] >= MAX_COST || costs[stc + 1] >= MAX_COST || costs[stc - 1] >= MAX_COST
                || costs[stcnx] >= MAX_COST || costs[stcnx + 1] >= MAX_COST || costs[stcnx - 1] >= MAX_COST
                || costs[stcpx] >= MAX_COST || costs[stcpx + 1] >= MAX_COST || costs[stcpx - 1] >= MAX_COST
                || oscillation_detected) {
            ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", costs[stc], (int) path.size());
            // check eight neighbors to find the lowest
            int minc = stc; //index of current cell
            int minp = costs[stc]; //costs of current cell
            //check each  neighbor cell in a counter clockwise fashion starting from the bottom cell
            int st = stcpx - 1; 
            if (costs[st] < minp) {
                minp = costs[st]; //
                minc = st;
            }
            st++;  
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st++; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st = stc - 1; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st = stc + 1; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st = stcnx - 1; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st++; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            st++; 
            if (costs[st] < minp) {
                minp = costs[st];
                minc = st;
            }
            stc = minc;
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    costs[stc], path[npath-1].first, path[npath-1].second);

            if (costs[stc] >= MAX_COST) { //surrounded by obstacles
                ROS_DEBUG("[PathCalc] No path found, high costs");
                //savemap("navfn_highpot");
                return 0;
            }
        }

        // 
        // else current cell neighbors are free (not an obstacle or over inflated)
        else {

            // get grad at current cell, top cell, top-right, and right.
            gradCell(costs, stc);
            gradCell(costs, stc + 1);
            gradCell(costs, stcnx);
            gradCell(costs, stcnx + 1);

            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // check for zero gradient, failed
            if (x == 0.0 && y == 0.0) {
                ROS_DEBUG("[PathCalc] Zero gradient");
                return 0;
            }

            // move in the right direction
            float ss = pathStep_ / std::hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // check for overflow
            // if overflow, then move the index in that direction
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += nx_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= nx_;
                dy += 1.0;
            }

        }

        // printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
                // costs[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down
float GradientPathTraceback::gradCell(std::vector<double> &costs, int n) 
{
    if (gradx_[n] + grady_[n] > 0.0)    // check if value is already assigned
        return 1.0;

    if (n < nx_ || n > nx_ * ny_ - nx_)    // check if index is out of bounds
        return 0.0;
    float cv = costs[n]; 
    float dx = 0.0;
    float dy = 0.0;

    // if current cell is "in an obstacle"
    // then assign a lethal cost to dx and dy
    if (cv >= MAX_COST) {
        if (costs[n - 1] < MAX_COST) //left and right
            dx = -costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        else if (costs[n + 1] < MAX_COST)
            dx = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

        if (costs[n - nx_] < MAX_COST)  //up and down
            dy = -costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        else if (costs[n + nx_] < MAX_COST)
            dy = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }

    else   // not in an obstacle
    {
        // assign to dx and dy the difference in cost between current cell and neighboring cell
        if (costs[n - 1] < MAX_COST) 
            dx += costs[n - 1] - cv;
        if (costs[n + 1] < MAX_COST)
            dx += cv - costs[n + 1];

        if (costs[n - nx_] < MAX_COST)
            dy += costs[n - nx_] - cv;
        if (costs[n + nx_] < MAX_COST)
            dy += cv - costs[n + nx_];
    }

    // normalize
    //then assign gradient value to the gradient array
    float norm = hypot(dx, dy); //returns hypothenuse
    if (norm > 0) 
    { 
        norm = 1.0 / norm; 
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}


} //end namespace global_planner

