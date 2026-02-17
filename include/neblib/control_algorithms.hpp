#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <optional>

#include "neblib/util.hpp"

namespace neblib
{
    class BaseController
    {
    public:
        /// @brief Virtual destructor for the base controller.
        ///
        /// Ensures that derived controller objects are correctly destroyed
        /// when deleted through a pointer to BaseController.
        /// This is important for proper cleanup of any resources managed
        /// by derived classes.
        virtual ~BaseController() = default;

        /// @brief Compute the control output for a given error value
        ///
        /// The function takes the current error (difference between setpoint and measurement)
        /// and optionally clamps the output to a specified range.
        ///
        /// @param error The current error input for the controller.
        /// @param clamp Optional array specifying the minimum and maximum output values.
        ///              Defaults to {-∞, ∞}, meaning no clamping.
        ///              The first element is the minimum output, the second is the maximum.
        /// @return The computed control output, potentially clamped.
        /// @note Derived classes must implement this function according to their specific control algorithm.
        virtual double getOutput(
            const double error,
            const std::array<double, 2> clamp = {
                -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()}) = 0;

        /// @brief Determine if the controller has settled.
        ///
        /// The function determines if the specified controller has settled or ended.
        ///
        /// @return true if settled, false otherwise.
        /// @note Derived classes must implement this function according to their specific control algorithm.
        virtual bool isSettled() = 0;

        /// @brief Resets the controller state to initial conditions.
        ///
        /// Typically called when a new target is set or after a manual override.
        /// Derived classes should reset any internal integrators, filters, or state.
        ///
        /// @note Derived classes must implement this function according to their specific control algorithm.
        virtual void reset() = 0;
    };

    class PIDController : public BaseController
    {
    private:
        // PID Gains
        const double kP;
        const double kI;
        const double kD;
        const double windupRange;
        const bool resetWindupOnSignChange;

        // PID Exit Condition
        const double tolerance;
        const unsigned int settleTime;
        const unsigned int iterationTime;

        double integral;
        std::optional<double> previousError;
        unsigned int timeSettled;

    public:
        /// @brief Construct a PIDController with specified gains and configuration.
        ///
        /// Initializes the PID controller with proportional, integral, and derivative gains,
        /// as well as limits for integer windup, output tolerance, and iteration timing.
        /// Optionally, the controller can reset the integrator when the error changes sign.
        ///
        /// @param kP Proportional gain.
        /// @param kI Integral gain.
        /// @param kD Derivative gain.
        /// @param windupRange Maximum magnitude for integral windup.
        /// @param tolerance Error tolerance to consider the system "at target".
        /// @param settleTime Minimum time the error must stay within tolerance to consider "settled".
        /// @param iterationTime Iteration time in milliseconds for the control loop.
        /// @param resetWindupOnSignChange Whether to reset the integral component when error changes sign. Defaults to true.
        PIDController(
            const double kP,
            const double kI,
            const double kD,
            const double windupRange,
            const double tolerance,
            const unsigned int settleTime,
            const unsigned int iterationTime,
            const bool resetWindupOnSignChange = true);

        /// @brief Compute the PID output for the given error value.
        ///
        /// The function takes the current error (difference between setpoint and measurement)
        /// and optionally clamps the output to a specified range.
        ///
        /// @param error The current error input for the controller.
        /// @param clamp Optional array specifying the minimum and maximum output values.
        ///              Defaults to {-∞, ∞}, meaning no clamping.
        /// @return The computed PID output, potentially clamped.
        double getOutput(
            const double error,
            const std::array<double, 2> clamp = {
                -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()}) override;

        /// @brief Determine if the PID has settled.
        ///
        /// The function determines if PID controller has settled or ended.
        ///
        /// @return true if settled, false otherwise.
        bool isSettled() override;

        /// @brief Resets the PID state to initial conditions.
        ///
        /// Typically called when a new target is set or after a manual override.
        void reset() override;
    };
}