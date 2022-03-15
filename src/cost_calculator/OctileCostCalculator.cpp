#include <cost_calculator/OctileCostCalculator.h>

namespace global_planner {

    float OctileCostCalculator::calcDist(const int &idx1, const int &idx2, const int cost_state)
    {
        int mx1; int my1;
        int mx2; int my2;
        
        indexToMap(idx1, mx1, my1);
        indexToMap(idx2, mx2, my2);

        int dx =std::abs(mx2 - mx1);
        int dy =std::abs(my2 - my1);

        int L = std::max(dx, dy);
        int S = std::min(dx, dy);

        float dist = SQRT2 * S + (L - S);

        return dist * cost_state;
    }
    
    float OctileCostCalculator::calcHCost_g(const int &idx1)
    {

        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx =std::abs(gx_ - mx);
        int dy =std::abs(gy_ - my);

        int L = std::max(dx, dy);
        int S = std::min(dx, dy);

        float dist = SQRT2 * S + (L - S);

        return dist * neutral_cost_;
    }

    float OctileCostCalculator::calcHCost_s(const int &idx1)
    {
        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx =std::abs(sx_ - mx);
        int dy =std::abs(sy_ - my);

        int L = std::max(dx, dy);
        int S = std::min(dx, dy);

        float dist = SQRT2 * S + (L - S);

        return dist * neutral_cost_;
    }


} //end namespace global_planner