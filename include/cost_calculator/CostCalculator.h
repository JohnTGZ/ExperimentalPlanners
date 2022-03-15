#ifndef _COSTCALCULATOR_H
#define _COSTCALCULATOR_H

#include <cmath>
#include <algorithm>

namespace global_planner {

class CostCalculator {
    public:

        CostCalculator(){}
        CostCalculator(const int nx, const int ny, const int ns, const int neutral_cost)
        {
            nx_ = nx;
            ny_ = ny;
            ns_ = ns;
            neutral_cost_ = neutral_cost;
        }

        void setStartGoal(const int sx, const int sy, const int gx, const int gy)
        {
            sx_ = sx;
            sy_ = sy;
            gx_ = gx;
            gy_ = gy;
        }

        virtual float calcDist(const int &idx1, const int &idx2, const int cost_state) {};
        virtual float calcHCost_g(const int &idx1){};
        virtual float calcHCost_s(const int &idx1){};
    protected:
        void indexToMap(const int &idx, int &mx, int &my)
        {
            my = idx/nx_;
            mx = idx - my * nx_;
        }

        void mapToIndex(int &idx, const int &mx, const int &my)
        {
            idx =  mx + (my * nx_);
        }
        
        int nx_, ny_, ns_;
        int sx_, sy_, gx_, gy_;
        int neutral_cost_;
};

} //end namespace global_planner

#endif