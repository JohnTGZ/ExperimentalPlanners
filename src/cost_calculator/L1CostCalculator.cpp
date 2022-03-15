#include <cost_calculator/L1CostCalculator.h>

namespace global_planner {

    float L1CostCalculator::calcDist(const int &idx1, const int &idx2, const int cost_state)
    {
        int mx1; int my1;
        int mx2; int my2;
        
        indexToMap(idx1, mx1, my1);
        indexToMap(idx2, mx2, my2);

        int dx =std::abs(mx2 - mx1);
        int dy =std::abs(my2 - my1);

        return (dx + dy)*cost_state;  
    }
    
    float L1CostCalculator::calcHCost_g(const int &idx1)
    {
        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx = std::abs(gx_ - mx);
        int dy =std::abs(gy_ - my);

        return (dx + dy) * neutral_cost_;
    }

    float L1CostCalculator::calcHCost_s(const int &idx1)
    {
        int mx; int my;
        
        indexToMap(idx1, mx, my);

        int dx =std::abs(sx_ - mx);
        int dy =std::abs(sy_ - my);

        return (dx + dy) * neutral_cost_;
    }


} //end namespace global_planner