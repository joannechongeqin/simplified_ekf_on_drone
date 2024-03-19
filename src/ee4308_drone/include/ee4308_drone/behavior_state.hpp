#pragma once
namespace ee4308::drone
{
    enum class BehaviorState
    {
        Takeoff,
        Start,
        Turtle,
        Waypoint,
        Land,
        Shutdown,
    };

    std::ostream &operator<<(std::ostream &out, const BehaviorState &state)
    {
        if (state == BehaviorState::Takeoff)
            out << "Takeoff";
        else if (state == BehaviorState::Start)
            out << "Start";
        else if (state == BehaviorState::Turtle)
            out << "Turtle";
        else if (state == BehaviorState::Waypoint)
            out << "Waypoint";
        else if (state == BehaviorState::Land)
            out << "Land";
        else if (state == BehaviorState::Shutdown)
            out << "Shutdown";
        else
            out << "?UnknownState?";
        return out;
    }

}