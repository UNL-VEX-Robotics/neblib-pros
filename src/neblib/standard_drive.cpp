#include "standard_drive.hpp"

neblib::StandardDrive::StandardDrive(
    std::unique_ptr<pros::MotorGroup> leftMotors,
    std::unique_ptr<pros::MotorGroup> rightMotors,
    pros::Imu *imu,
    neblib::TrackerWheel *parallelTrackerWheel)
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
    pros::MotorGears leftGearset = leftMotors->get_gearing();
    pros::MotorGears rightGearset = rightMotors->get_gearing();

    switch (unit)
    {
    case percent:
        if (leftGearset == pros::MotorGears::invalid || rightGearset == pros::MotorGears::invalid)
            return;

        leftMotors->move_velocity(leftInput * static_cast<int>(leftGearset));
        rightMotors->move_velocity(rightInput * static_cast<int>(rightGearset));
        break;
    case rpm:
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

int neblib::StandardDrive::driveFor(
    const double distance,
    const int timeout,
    const double heading,
    const std::array<double, 2> clamp)
{
    if (!linearController || !angularController || !imu || !parallelTrackerWheel)
        return -1;

    linearController->reset();
    angularController->reset();

    const double targetPosition = parallelTrackerWheel->getPosition() + distance;
    int t = 0;
    while (!linearController->isSettled() && t < timeout)
    {
        const int linearOutput = static_cast<int>(linearController->getOutput(targetPosition - parallelTrackerWheel->getPosition(), clamp));
        const int angularOutput = static_cast<int>(angularController->getOutput(neblib::wrap(heading - imu->get_heading(), -180.0, 180.0), clamp));

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

int neblib::StandardDrive::driveFor(
    const double distance,
    const int timeout,
    const std::array<double, 2> clamp)
{
    return this->driveFor(distance, timeout, imu->get_heading(), clamp);
}

int neblib::StandardDrive::turnFor(const double degrees, const int timeout, const std::array<double, 2> clamp)
{
    if (!angularController || !imu)
        return -1;

    angularController->reset();

    const double target = imu->get_rotation() + degrees;
    int t = 0;
    while (!angularController->isSettled() && t < timeout)
    {
        const int output = static_cast<int>(angularController->getOutput(target - imu->get_rotation(), clamp));

        this->arcadeDrive(
            0.0,
            output,
            neblib::millivolt);

        pros::delay(10);
        t += 10;
    }

    this->stop(pros::MotorBrake::hold);
    return t;
}

int neblib::StandardDrive::turnTo(const double heading, const int timeout, const std::array<double, 2> clamp)
{
    if (!angularController || !imu)
        return -1;

    angularController->reset();

    int t = 0;
    while (!angularController->isSettled() && t < timeout)
    {
        const int output = static_cast<int>(angularController->getOutput(neblib::wrap(heading - imu->get_heading(), -180.0, 180.0), clamp));

        this->arcadeDrive(
            0.0,
            output,
            neblib::millivolt);

        pros::delay(10);
        t += 10;
    }

    this->stop(pros::MotorBrake::hold);
    return t;
}

int neblib::StandardDrive::swingFor(const neblib::TurnDirection direction, const double degrees, const int timeout, const std::array<double, 2> clamp)
{
    if (!angularController || !imu)
        return -1;

    angularController->reset();

    const double target = (direction == neblib::TurnDirection::right ? imu->get_rotation() + degrees : imu->get_rotation() - degrees);
    int t = 0;

    if (direction == neblib::TurnDirection::right)
    {
        while (!angularController->isSettled())
        {
            const int output = static_cast<int>(angularController->getOutput(target - imu->get_rotation(), clamp));

            rightMotors->set_brake_mode_all(pros::MotorBrake::hold);
            rightMotors->brake();
            leftMotors->move_voltage(output);

            pros::delay(10);
            t += 10;
        }
    }
    else
    {
        while (!angularController->isSettled())
        {
            const int output = static_cast<int>(angularController->getOutput(imu->get_rotation() - target, clamp));

            leftMotors->set_brake_mode_all(pros::MotorBrake::hold);
            leftMotors->brake();
            rightMotors->move_voltage(output);

            pros::delay(10);
            t += 10;
        }
    }

    this->stop(pros::MotorBrake::hold);
    return t;
}

int neblib::StandardDrive::swingTo(const neblib::TurnDirection direction, const double heading, const int timeout, const std::array<double, 2> clamp)
{
    if (!angularController || !imu)
        return -1;

    angularController->reset();

    int t = 0;

    if (direction == neblib::TurnDirection::right)
    {
        while (!angularController->isSettled())
        {
            const int output = static_cast<int>(angularController->getOutput(neblib::wrap(heading - imu->get_heading(), -180.0, 180.0), clamp));

            rightMotors->set_brake_mode_all(pros::MotorBrake::hold);
            rightMotors->brake();
            leftMotors->move_voltage(output);

            pros::delay(10);
            t += 10;
        }
    }
    else
    {
        while (!angularController->isSettled())
        {
            const int output = static_cast<int>(angularController->getOutput(neblib::wrap(imu->get_heading() - heading, -180.0, 180.0), clamp));

            leftMotors->set_brake_mode_all(pros::MotorBrake::hold);
            leftMotors->brake();
            rightMotors->move_voltage(output);

            pros::delay(10);
            t += 10;
        }
    }

    this->stop(pros::MotorBrake::hold);
    return t;
}
