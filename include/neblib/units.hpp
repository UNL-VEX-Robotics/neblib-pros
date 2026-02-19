#pragma once

namespace neblib
{
    enum VelocityUnits
    {
        percent = 0,
        pct = percent,
        rpm = 1,
        volt = 2,
        mV = 3,
        millivolt = mV
    };

    enum TurnDirection
    {
        left = 0,
        right = 1
    };

    enum DriveDirection
    {
        forward = 0,
        fwd = forward,
        reverse = 1,
        rev = reverse
    };
}