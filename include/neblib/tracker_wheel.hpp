#pragma once

#include "api.h"

namespace neblib
{
    class TrackerWheel
    {
    public:
        /// @brief Virtual destructor for the base tracker wheel.
        ///
        /// Ensures that derived tracker wheel objects are correctly destroyed
        /// when deleted through a pointer to TrackerWheel.
        /// This is important for proper cleanup of any resources managed
        /// by derived classes.
        virtual ~TrackerWheel() = default;

        /// @brief Compute the current position of the tracker wheel.
        ///
        /// The function calculates the tracker wheel's position in the same unit as the wheel diameter.
        ///
        /// @return The position in the same units as wheelDiameter.
        /// @note Derived classes must implement this function according to their specific tracker wheel sensor.
        virtual double getPosition() = 0;

        /// @brief Resets the position of the tracker wheel.
        ///
        /// The function resets the position of the tracker wheel.
        /// Generally called before a position tracking algorithm starts.
        ///
        /// @note Derived classes must implement this function according to their specific tracker wheel sensor.
        virtual void reset() = 0;
    };

    class RotationTrackerWheel : public TrackerWheel
    {
    private:
        std::unique_ptr<pros::Rotation> rotation;
        const double wheelDiameter;

    public:
        /// @brief Creates a RotationTrackerWheel object.
        ///
        /// Initializes a RotationTrackerWheel object using a rotation sensor and wheel diameter.
        ///
        /// @param rotation Unique pointer to a rotation sensor.
        /// @param wheelDiameter Diameter of the tracker wheel.
        RotationTrackerWheel(std::unique_ptr<pros::Rotation> rotation, const double wheelDiameter);

        /// @brief Compute the current position of the tracker wheel.
        ///
        /// The function calculates the tracker wheel's position in the same unit as the wheel diameter.
        ///
        /// @return The position in the same units as wheelDiameter.
        double getPosition() override;

        /// @brief Resets the position of the tracker wheel.
        ///
        /// The function resets the position of the tracker wheel.
        /// Generally called before a position tracking algorithm starts.
        void reset() override;
    };
}