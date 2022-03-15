#ifndef _L1COSTCALCULATOR_H
#define _L1COSTCALCULATOR_H

#include <cost_calculator/CostCalculator.h>
#include <cmath>

namespace global_planner {

class L1CostCalculator : public CostCalculator {
    public:

        L1CostCalculator(const int nx, const int ny, const int ns, const int neutral_cost): 
            CostCalculator(nx, ny, ns, neutral_cost) {}

        float calcDist(const int &idx1, const int &idx2, const int cost_state);
        float calcHCost_g(const int &idx1);
        float calcHCost_s(const int &idx1);
};

} //end namespace global_planner

#endif