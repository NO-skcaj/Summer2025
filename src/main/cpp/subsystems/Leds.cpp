#include "subsystems/Leds.h"


using namespace std;

Leds* Leds::GetInstance()
{
    static Leds leds;
    return &leds;
}

/// @brief Class to support an addressable LED string.
Leds::Leds() : m_led          {Constants::Led::PwmPort},
               m_shooting     {frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, 
                                   std::array<frc::Color, 2>{frc::Color::kRed, frc::Color::kBlack}
                                   ).ScrollAtAbsoluteSpeed(0.5_mps, Constants::Led::LedSpacing)},
               m_scrollingRainbow{frc::LEDPattern::Rainbow(255, 128).ScrollAtAbsoluteSpeed(0.5_mps, Constants::Led::LedSpacing)},

               m_ledMode      {LedMode::Rainbow},

               m_firstPixelHue{0},
               m_cycleCounter {0}
{
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(Constants::Led::Length);

    // Set the default mode
    SetMode(LedMode::Rainbow);

    // Intialize the LED data
    m_led.SetData(m_ledBuffer);

    // Start the addressable LED communications
    m_led.Start();
}

/// @brief This method will be called once periodically.
void Leds::Periodic()
{
    switch (m_ledMode)
    {
        case LedMode::Off:
        case LedMode::SolidGreen:
        case LedMode::SolidRed:
            break;

        case LedMode::HvaColors:
            HvaColors();
            break;

        case LedMode::Strobe:
            Strobe();
            break;

        case LedMode::ShootingAnimation:
            // Apply the shootime pattern to the data buffer
            m_shooting.ApplyTo(m_ledBuffer);
            break;

        case LedMode::Rainbow:
            // Run the rainbow pattern and apply it to the buffer
            m_scrollingRainbow.ApplyTo(m_ledBuffer);
            break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

/// @brief Setting the Led's mode to the given parameter.
/// @param ledMode mode to set the Leds.
void Leds::SetMode(LedMode ledMode)
{
    // Remember the LED mode
    m_ledMode = ledMode;

    // Set the LEDs based on the LED mode
    switch (m_ledMode)
    {
    case LedMode::Off:
        SolidColor(0, 0, 0);
        break;

    case LedMode::SolidGreen:
        SolidColor(0, Constants::Led::Green, 0);
        break;

    case LedMode::SolidRed:
        SolidColor(Constants::Led::Red, 0, 0);
        break;

    case LedMode::HvaColors:
        m_cycleCounter = 0;
        HvaColors();
        break;

    case LedMode::Strobe:
        m_cycleCounter = 0;
        Strobe();
        break;

    default:
        break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

/// @brief Method to support setting the LED string to the specified solid color.
/// @param red The red component of the LED color.
/// @param green The green component of the LED color.
/// @param blue The blue component of the LED color.
void Leds::SolidColor(int red, int green, int blue)
{
    // Set the value for every pixel
    for (auto ledIndex = 0; ledIndex < Constants::Led::Length; ledIndex++)
        m_ledBuffer[ledIndex].SetRGB(red * Constants::Led::Brightness, green * Constants::Led::Brightness, blue * Constants::Led::Brightness);
}

/// @brief Method to support setting the LED string to HVA alternating color.
void Leds::HvaColors()
{
    int firstColor  = Constants::Led::Blue;
    int secondColor = 0;

    // Alternate the colors
    if (m_cycleCounter % Constants::Led::HvaDelay < Constants::Led::HvaDelay / 2)
    {
        firstColor  = 0;
        secondColor = Constants::Led::Blue;
    }

    // For every pixel
    for (auto ledIndex = 0; ledIndex < Constants::Led::Length; ledIndex++)
    {
        // Set the color based on the pixel index
        if (ledIndex % 2 == 0)
            m_ledBuffer[ledIndex].SetRGB(0, 0, firstColor * Constants::Led::Brightness);
        else
            m_ledBuffer[ledIndex].SetRGB(0, 0, secondColor * Constants::Led::Brightness);
    }

    // Update the cycle counter
    m_cycleCounter++;
}

/// @brief Method to strobe the LED string.
void Leds::Strobe()
{
    if (m_cycleCounter % Constants::Led::StrobeDelay == 0)
        SolidColor(Constants::Led::Red, Constants::Led::Green, Constants::Led::Blue);
    else
        SolidColor(0, 0, 0);

    // Update the cycle counter
    m_cycleCounter++;
}
