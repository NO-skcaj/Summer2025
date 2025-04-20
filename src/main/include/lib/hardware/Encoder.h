#include <units/angle.h>

class Encoder
{
public:
    virtual units::turn_t GetAbsoluteValue() = 0;
};