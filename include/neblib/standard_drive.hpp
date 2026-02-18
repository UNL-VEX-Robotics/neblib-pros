#pragma once

#include "api.h"
#include "neblib/control_algorithms.hpp"
#include "neblib/tracker_wheel.hpp"
#include "neblib/units.hpp"

namespace neblib
{
    class StandardDrive
    {
    private:
        std::unique_ptr<pros::MotorGroup> leftMotors;
        std::unique_ptr<pros::MotorGroup> rightMotors;

        pros::Imu &imu;
        neblib::TrackerWheel &parallelTrackerWheel;

        std::unique_ptr<neblib::BaseController> linearController;
        std::unique_ptr<neblib::BaseController> angularController;
        std::unique_ptr<neblib::BaseController> swingController;

    public:
        /// @brief Constructs a StandardDrive object.
        ///
        /// Initializes a StandardDrive object using motor groups for left and right, an imu, and a tracking wheel.
        ///
        /// @param leftMotors The left side motors
        /// @param rightMotors The right side motors
        /// @param imu A VEX V5 Inertial Sensor
        /// @param parallelTrackerWheel A neblib::TrackingWheel or derived class
        StandardDrive(
            std::unique_ptr<pros::MotorGroup> leftMotors,
            std::unique_ptr<pros::MotorGroup> rightMotors,
            pros::Imu &imu,
            neblib::TrackerWheel &parallelTrackerWheel);

        /// @brief Sets the control algorithms used for autonomous movements.
        ///
        /// The function takes unique pointers for the different controllers.
        /// These controllers are used when making autonomous movements.
        ///
        /// @param linearController The controller used for driving
        /// @param angularController The controller used for turning
        /// @param swingController The controller used for swing movements.
        void setControllers(
            std::unique_ptr<neblib::BaseController> linearController,
            std::unique_ptr<neblib::BaseController> angularController,
            std::unique_ptr<neblib::BaseController> swingController);

        /// @brief Sets the control algorithm for drive movements.
        ///
        /// The function takes a unique pointer for the linear controller.
        ///
        /// @param linearController The controller used for driving
        void setLinearController(std::unique_ptr<neblib::BaseController> linearController);

        /// @brief Sets the control algorithm for turn movements.
        ///
        /// The function takes a unique pointer for the angular controller.
        ///
        /// @param angularController The controller used for turning
        void setAngularController(std::unique_ptr<neblib::BaseController> angularController);

        /// @brief Sets the control algorithm for swing movements.
        ///
        /// The function takes a unique pointer for the swing controller.
        ///
        /// @param swingController The controller used for swing movements
        void setSwingController(std::unique_ptr<neblib::BaseController> swingController);

        /// @brief Controls the drive motors with independent left and right inputs.
        ///
        /// The function takes left and right inputs and an optional unit type.
        /// Unit types percent and rpm use `pros::MotorGroup::move_velocity()`.
        ///
        /// @param leftInput Input for the left side of the drive
        /// @param rightInput Input for the right side of the drive
        /// @param unit The unit of the inputs, defaults to volt
        void tankDrive(
            const int leftInput,
            const int rightInput,
            const neblib::VelocityUnits unit = neblib::VelocityUnits::volt);

        /// @brief Controls the drive motors with linear and angular inputs.
        ///
        /// The function takes linear and angular inputs and an optional unit type.
        /// Unit types percent and rpm use `pros::MotorGroup::move_velocity()`.
        ///
        /// @param linearInput Input to control forward and backwards
        /// @param angularInput Input to control turning
        /// @param unit The unit of the inputs, defaults to volt
        void arcadeDrive(
            const int linearInput,
            const int angularInput,
            const neblib::VelocityUnits unit = neblib::VelocityUnits::volt);

        /// @brief Stops the motors using a specified brake type.
        ///
        /// The function optionally takes a brake type, defualts to hold.
        ///
        /// @param brakeType Defaults to hold
        void stop(pros::MotorBrake brakeType = pros::MotorBrake::hold);

        /// @brief Autonomously drives a specified distance.
        ///
        /// The function takes a distance and heading, and optionally a clamp for min and max output.
        /// Relies on both linearController and angularController.
        /// Units of the distance are based on the units of the tracking wheel.
        ///
        /// @param distance The distance to be driven
        /// @param timeout The amount of time before the move is forced to end, mS
        /// @param heading The heading the robot will try to maintain
        /// @param clamp Optional array specifying the minimum and maximum output values.
        ///              Defaults to {-∞, ∞}, meaning no clamping.
        /// @return The amount of time the movement took, mS.
        ///         Returns -1 if there is no specified control algorithm
        int driveFor(
            const double distance,
            const int timeout,
            const double heading,
            const std::array<double, 2> clamp = {
                -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()});

        /// @brief Autonomously drives a specified distance.
        ///
        /// The function takes a distance and optionally a clamp for min and max output.
        /// Relies on both linearController and angularController.
        /// Units of the distance are based on the units of the tracking wheel.
        /// Holds the current heading of the robot.
        ///
        /// @param distance The distance to be driven
        /// @param timeout The amount of time before the move is forced to end, mS
        /// @param clamp Optional array specifying the minimum and maximum output values.
        ///              Defaults to {-∞, ∞}, meaning no clamping.
        /// @return The amount of time the movement took, mS.
        ///         Returns -1 if there is no specified control algorithm
        int driveFor(
            const double distance,
            const int timeout,
            const std::array<double, 2> clamp = {
                -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()});

    };
}