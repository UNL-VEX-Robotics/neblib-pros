#include "standard_drive.hpp"

neblib::StandardDrive::StandardDrive(
    std::unique_ptr<pros::MotorGroup> leftMotors,
    std::unique_ptr<pros::MotorGroup> rightMotors,
    pros::Imu &imu,
    neblib::TrackerWheel &parallelTrackerWheel)
    : leftMotors(std::move(leftMotors)),
      rightMotors(std::move(rightMotors)),
      imu(imu),
      parallelTrackerWheel(parallelTrackerWheel),
      linearController(nullptr),
      angularController(nullptr),
      swingController(nullptr)
{
}

void neblib::StandardDrive::setControllers(
    std::unique_ptr<neblib::BaseController> linearController,
    std::unique_ptr<neblib::BaseController> angularController,
    std::unique_ptr<neblib::BaseController> swingController)
{
    this->linearController = std::move(linearController);
    this->angularController = std::move(angularController);
    this->swingController = std::move(swingController);
}

void neblib::StandardDrive::setLinearController(std::unique_ptr<neblib::BaseController> linearController)
{
    this->linearController = std::move(linearController);
}

void neblib::StandardDrive::setAngularController(std::unique_ptr<neblib::BaseController> angularController)
{
    this->angularController = std::move(angularController);
}

void neblib::StandardDrive::setSwingController(std::unique_ptr<neblib::BaseController> swingController)
{
    this->swingController = std::move(swingController);
}

void neblib::StandardDrive::tankDrive(
    const int leftInput,
    const int rightInput,
    const neblib::VelocityUnits unit)
{
    switch (unit)
    {
    case percent:
        pros::MotorGears leftGearset = leftMotors->get_gearing();
        pros::MotorGears rightGearset = rightMotors->get_gearing();
        if (leftGearset == pros::MotorGears::invalid || rightGearset == pros::MotorGears::invalid)
            return;

        leftMotors->move_velocity(leftInput * static_cast<int>(leftGearset));
        rightMotors->move_velocity(rightInput * static_cast<int>(rightGearset));
        break;

    case rpm:
        pros::MotorGears leftGearset = leftMotors->get_gearing();
        pros::MotorGears rightGearset = rightMotors->get_gearing();
        if (leftGearset == pros::MotorGears::invalid || rightGearset == pros::MotorGears::invalid)
            return;

        leftMotors->move_velocity(leftInput);
        rightMotors->move_velocity(rightInput);
        break;

    case volt:
        leftMotors->move(leftInput);
        rightMotors->move(rightInput);
        break;

    case millivolt:
        leftMotors->move_voltage(leftInput);
        rightMotors->move_voltage(rightInput);
        break;
    }
}

void neblib::StandardDrive::arcadeDrive(
    const int linearInput,
    const int angularInput,
    const neblib::VelocityUnits unit)
{
    this->tankDrive(
        linearInput + angularInput,
        linearInput - angularInput,
        unit);
}

void neblib::StandardDrive::stop(pros::MotorBrake brakeType)
{
    leftMotors->set_brake_mode_all(brakeType);
    rightMotors->set_brake_mode_all(brakeType);
    leftMotors->brake();
    rightMotors->brake();
}

int neblib::StandardDrive::driveFor(const double distance, const double heading, const std::array<double, 2> clamp)
{
    if (!linearController || !angularController)
        return -1;

    linearController->reset();

    const double targetPosition = parallelTrackerWheel.getPosition() + distance;

    int t = 0;
    while (!linearController->isSettled())
    {
        const int linearOutput = static_cast<int>(linearController->getOutput(targetPosition - parallelTrackerWheel.getPosition(), clamp));
        const int angularOutput = static_cast<int>(angularController->getOutput(neblib::wrap(heading - imu.get_heading(), -180.0, 180.0), clamp));

        this->arcadeDrive(
            linearOutput,
            angularOutput,
            neblib::millivolt);

        pros::delay(10);
        t += 10;
    }

    this->stop(pros::MotorBrake::hold);
    return t;
}

int neblib::StandardDrive::driveFor(const double distance, const std::array<double, 2> clamp)
{
    return this->driveFor(distance, imu.get_heading(), clamp);
}
