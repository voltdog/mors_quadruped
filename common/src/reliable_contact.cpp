#include "reliable_contact.hpp"

#include "structs.hpp"

#include <cmath>

namespace
{

// Доля опорной фазы, за которую доверие плавно включается и выключается.
constexpr double kTransitionWidth = 0.2;

}  // namespace

float ReliableContact::get_trust_coefficient(double s_i, int state) const noexcept
{
    // Некорректная фаза не должна давать вес кинематическому ограничению.
    if (!std::isfinite(s_i) || s_i < 0.0 || s_i > 1.0)
    {
        return 0.0F;
    }

    // Доверие включается только для штатной опорной фазы.
    switch (state)
    {
        case STANCE:
            break;
        case SWING:
        case LATE_CONTACT:
        case EARLY_CONTACT:
        default:
            return 0.0F;
    }

    if (s_i < kTransitionWidth)
    {
        return static_cast<float>(s_i / kTransitionWidth);
    }

    if (s_i <= 1.0 - kTransitionWidth)
    {
        return 1.0F;
    }

    return static_cast<float>((1.0 - s_i) / kTransitionWidth);
}
