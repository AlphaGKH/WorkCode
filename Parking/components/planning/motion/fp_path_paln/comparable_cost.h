#ifndef PLANNING_MOTION_FP_PATH_PLAN_COMPARABLE_COST_H_
#define PLANNING_MOTION_FP_PATH_PLAN_COMPARABLE_COST_H_

#include <cmath>

namespace planning {

class ComparableCost
{
public:
    ComparableCost() = default;

    ComparableCost(const bool& is_truncated, const double& smooth_cost, const double& safety_cost)
        : is_truncated(is_truncated), smooth_cost(smooth_cost), safety_cost(safety_cost){}

    ComparableCost(const ComparableCost &) = default;

    // 1: current > other, -1: current < other, 0: current == other
    int CompareTo(const ComparableCost &other) const{
        if(is_truncated){
            if(!other.is_truncated){
                 return 1; // current truncated and other not truncated
            }
        }
        else {
            if(other.is_truncated){
                return -1; // current not truncated and other truncated
            }
        }

        constexpr double kEpsilon = 1e-12;
        const double diff = safety_cost + smooth_cost - other.safety_cost - other.smooth_cost;

        if (std::fabs(diff) < kEpsilon) {
            return 0;
        } else if (diff > 0) {
            return 1;
        } else {
            return -1;
        }
    }

    ComparableCost operator+(const ComparableCost &other) {
        ComparableCost lhs = *this;
        lhs += other;
        return lhs;
    }
    ComparableCost &operator+=(const ComparableCost &other) {
        is_truncated = is_truncated || other.is_truncated;
        safety_cost += other.safety_cost;
        smooth_cost += other.smooth_cost;
        return *this;
    }
    bool operator>(const ComparableCost &other) const {
        return this->CompareTo(other) > 0;
    }
    bool operator>=(const ComparableCost &other) const {
        return this->CompareTo(other) >= 0;
    }
    bool operator<(const ComparableCost &other) const {
        return this->CompareTo(other) < 0;
    }
    bool operator<=(const ComparableCost &other) const {
        return this->CompareTo(other) <= 0;
    }

private:
    bool is_truncated = false;
    double smooth_cost = 0.0;
    double safety_cost = 0.0;


};

}


#endif
