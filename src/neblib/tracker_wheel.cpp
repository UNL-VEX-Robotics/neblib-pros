#include "tracker_wheel.hpp"

neblib::RotationTrackerWheel::RotationTrackerWheel(
    std::unique_ptr<pros::Rotation> rotation,
    const double wheelDiameter)
    : rotation(std::move(rotation)),
      wheelDiameter(wheelDiameter)
{
}

double neblib::RotationTrackerWheel::getPosition()
{
    if (!rotation)
        return 0.0;
    return static_cast<double>(rotation->get_position()) / 36000.0 * M_PI * wheelDiameter;
}

void neblib::RotationTrackerWheel::reset()
{
    if (rotation)
        rotation->reset_position();
}
