#include<iostream>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>

#include <cmath>
#define M_PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;

struct State
{
    State(double x, double y, double yaw, int time = 0) : x(x), y(y), yaw(yaw), time(time)
    {
    }

    State() = default;

    bool operator==(const State &s) const
    {
        return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
    }

    State(const State &) = default;
    State(State &&) = default;
    State &operator=(const State &) = default;
    State &operator=(State &&) = default;

    friend std::ostream &operator<<(std::ostream &os, const State &s)
    {
        return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")";
    }

    double x;
    double y;
    double yaw;
    int time;
};


ompl::base::DubinsStateSpace::DubinsPath findDubins(State &start, State &goal)
{
    ompl::base::DubinsStateSpace dubinsSpace(1.0);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);
    dubinsEnd->setXY(goal.x, goal.y);
    dubinsEnd->setYaw(-goal.yaw);
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath = dubinsSpace.dubins(dubinsStart, dubinsEnd);

    for (auto pathidx = 0; pathidx < 3; pathidx++)
    {
        switch (dubinsPath.type_[pathidx])
        {
            case 0:  // DUBINS_LEFT
                std::cout << "Left" << std::endl;
                break;
            case 1:  // DUBINS_STRAIGHT
                std::cout << "Straight" << std::endl;
                break;
            case 2:  // DUBINS_RIGHT
                std::cout << "Right" << std::endl;
                break;
            default:
                std::cout << "\033[1m\033[31m"
                          << "Warning: Receive unknown DubinsPath type"
                          << "\033[0m\n";
                break;
        }
        std::cout << fabs(dubinsPath.length_[pathidx]) << std::endl;
    }

    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    return dubinsPath;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "hi bmo" << std::endl;
    
    State start(1, 1, 0);
    State goal(17, 17, -1 * M_PI);

    findDubins(start, goal);


    return 0;
}