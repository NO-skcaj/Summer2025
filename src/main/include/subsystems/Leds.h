#pragma once

#include <array>

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/LEDPattern.h>

#include "Constants/Controller.h"


namespace LedConstants
{
    constexpr auto PwmPort     =   9;

    constexpr auto Length      = 400;  // The length of the LED string
    constexpr auto Brightness  = 0.5;

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  20;  // The delay between strobe flashes
    constexpr auto HvaDelay    =  20;  // The delay between HVA color changes
}

/// @brief modes for the LED string.
enum LedMode
{
    Off,
    SolidGreen,
    SolidRed,
    HvaColors,
    Strobe,
    ShootingAnimation,
    Rainbow
};

class Leds : public frc2::SubsystemBase
{
    public:

        explicit Leds();

        void     Periodic() override;
        void     SetMode(LedMode ledMode);

    private:

        void Off();
        void SolidColor(int red, int green, int blue);
        void Rainbow();
        void HvaColors();
        void Strobe();
        void ShootingAnimation();

        LedMode             m_ledMode;            // The LED mode

        int                 m_firstPixelHue = 0;  // Store the hue of the first pixel for rainbow mode
        int                 m_cycleCounter  = 0;  // Counter for dynamic LED modes

        // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
        // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
        frc::LEDPattern     m_scrollingRainbow = frc::LEDPattern::Rainbow(255, 128).ScrollAtAbsoluteSpeed(0.1_mps, units::meter_t{1 / 120.0});

        // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
        // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
        frc::LEDPattern     m_shooting = frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, std::array<frc::Color, 2>{frc::Color::kRed, frc::Color::kBlack}).
                                                          ScrollAtAbsoluteSpeed(0.5_mps, units::meter_t{1 / 120.0});

        frc::AddressableLED m_led{LedConstants::PwmPort};

        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledBuffer;  // Instatntiate the LED data buffer
};
