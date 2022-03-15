#include <cost_calculator/ChebyshevCostCalculator.h>

namespace global_planner {

    float ChebyshevCostCalculator::calcDist(const int &idx1, const int &idx2, const int cost_state)
    {
        int mx1; int my1;
        int mx2; int my2;
        
        indexToMap(idx1, mx1, my1);
        indexToMap(idx2, mx2, my2);

        int dx =std::abs(mx2 - mx1);
        int dy =std::abs(my2 - my1);

        int max_ = std::max(dx, dy);

        return max_*cost_state;
    }
    
    float ChebyshevCostCalculator::calcHCost_g(const int &idx1)
    {
        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx =std::abs(gx_ - mx);
        int dy =std::abs(gy_ - my);

        int max_ = std::max(dx, dy);

        return max_*neutral_cost_;
    }

    float ChebyshevCostCalculator::calcHCost_s(const int &idx1)
    {
        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx =std::abs(sx_ - mx);
        int dy =std::abs(sy_ - my);

        int max_ = std::max(dx, dy);

        return max_*neutral_cost_;
    }


} //end namespace global_planner