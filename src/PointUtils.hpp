#pragma once

#include <LibGfx/Point.h>

namespace Gfx
{
    inline float dot(Point<float> p1, Point<float> p2)
    {
        return p1.x() * p2.x() + p1.y() * p2.y();
    }
}
