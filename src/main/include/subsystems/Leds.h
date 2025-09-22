#pragma once

#include <array>

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/LEDPattern.h>

#include "Constants/Led.h"


/// @brief modes for the LED string.
enum class LedMode
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
        static Leds* GetInstance();

        void     Periodic() override;
        void     SetMode(LedMode ledMode);

    private:

        Leds();

        void Off();
        void SolidColor(int red, int green, int blue);
        void Rainbow();
        void HvaColors();
        void Strobe();
        void ShootingAnimation();
        
        frc::AddressableLED m_led;

        // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
        // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
        frc::LEDPattern     m_shooting;
        
        // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
        // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
        frc::LEDPattern     m_scrollingRainbow;

        std::array<frc::AddressableLED::LEDData, Constants::Led::Length> m_ledBuffer;  // Instatntiate the LED data buffer

        LedMode             m_ledMode;        // The LED mode

        int                 m_firstPixelHue;  // Store the hue of the first pixel for rainbow mode
        int                 m_cycleCounter;   // Counter for dynamic LED modes
};
