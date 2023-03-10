#pragma once
#include <frc/AddressableLED.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/DigitalInput.h>
#include "System.hxx"
#include <array>

class LEDs : public System
{
public:
    LEDs();
    void setRange(int startIndex, int width, char iRed, char iGreen, char iBlue);
    void clear();
    void displayTeamColor();
    void displayFancyTeamColors();
    void displayRainbow();
    void displayFallingLights();
    void displayOrangeBlue();
    void displayRedWhiteAndBlue();
    void displayAutumnColors();
    void displayWinterColors();
    double lerp(double a, double b, double t);
    void updateLEDs();

    void signalCube();
    void signalCone();
    // virtual void updateSystem(double timestamp, char mode) override;

    static std::shared_ptr<LEDs> instance()
    {
        static std::shared_ptr<LEDs> leds = std::make_shared<LEDs>();
        return leds;
    }

private:
    static constexpr int kLength = 600;
    // // PWM port 9
    // // Must be a PWM header, not MXP or DIO
    // frc::AddressableLED m_led{0};
    std::array<frc::AddressableLED::LEDData, kLength>
        m_ledBuffer; // Reuse the buffer

    frc::Timer m_LEDTimer;
};
