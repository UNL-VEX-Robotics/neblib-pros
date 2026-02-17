#include "control_algorithms.hpp"

neblib::PIDController::PIDController(
    const double kP,
    const double kI,
    const double kD,
    const double windupRange,
    const double tolerance,
    const unsigned int settleTime,
    const unsigned int iterationTime,
    const bool resetWindupOnSignChange)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      resetWindupOnSignChange(resetWindupOnSignChange),
      tolerance(tolerance),
      settleTime(settleTime),
      iterationTime(iterationTime),
      integral(0.0),
      previousError(std::nullopt),
      timeSettled(0)
{
}

double neblib::PIDController::getOutput(
    const double error,
    const std::array<double, 2> clamp)
{
    // Update Integral
    if (std::abs(error) < windupRange)
        integral += error;
    else
        integral = 0;

    if (resetWindupOnSignChange && neblib::sign(error) != neblib::sign(previousError.value_or(error)))
        integral = 0;

    // Calculate Derivative
    const double derivative = error - previousError.value_or(error);

    // Update Settle State
    if (std::abs(error) < tolerance)
        timeSettled += iterationTime;

    // Update previousError
    previousError = error;

    // Calculate Output
    return std::clamp(error * kP + integral * kI + derivative * kD, clamp.at(0), clamp.at(1));
}

bool neblib::PIDController::isSettled()
{
    return timeSettled >= settleTime;
}

void neblib::PIDController::reset()
{
    integral = 0.0;
    previousError.reset();
    timeSettled = 0;
}