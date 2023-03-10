#include "LEDs.hxx"

#include "Controllers.hxx"

#if 0
LEDs::LEDs()
{
    m_LEDTimer.Reset();
    m_LEDTimer.Start();

    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

}


void LEDs::displayRainbow()
{
    int spacing = 1;

    for (int i = 0; i < kLength; i++)
    {
        // Alternate sin function movement
        // int shift = (int)(sin(m_timer.Get()) * kLength / 2 + kLength / 2);

        // Shift at a rate of 10 lights per seconds
        int shift = (int)(m_LEDTimer.Get() * 10);
        // Shift the lights, duplicate at each spacing interval, and wrap the value to the length of the LED array
        int offset = (shift % kLength) % spacing;
        // Final value to determine if an LED should be lit. Separated from the if statement for tidyness
        bool shouldLight = (i % spacing) == offset;
        if (shouldLight)
        {
            m_ledBuffer[i].SetHSV(
                (int)((i / (float)kLength) * 180 + ((int)(m_LEDTimer.Get() * 120))) % 180, // Rotating hue, 120 degrees a second
                255,                                                                       // Max shade
                190);                                                                      // 190 value
        }
        else
        {
            m_ledBuffer[i].SetRGB(0, 0, 0); // idk what
        }
    }
    m_led.SetData(m_ledBuffer); // This turns the lights on
}

void LEDs::displayTeamColor()
{
    auto alliance = frc::DriverStation::GetAlliance();
    for (int i = 0; i < kLength; i++)
    {
        if (alliance == frc::DriverStation::kRed)
        {
            m_ledBuffer[i].SetRGB(200, 0, 0); // Red
        }
        else
        {
            m_ledBuffer[i].SetRGB(0, 0, 200); // Blue
        }
    }
    m_led.SetData(m_ledBuffer); // This turns the lights on
}

void LEDs::displayFancyTeamColors()
{
    auto alliance = frc::DriverStation::GetAlliance();

    if (alliance == frc::DriverStation::kRed)
    {
        LEDs::displayAutumnColors(); // Red
    }
    else
    {
        LEDs::displayWinterColors(); // Blue
    }
}
void LEDs::displayFallingLights()
{
    int spacing = 3;

    for (int i = 0; i < kLength; i++)
    {
        // Alternate sin function movement
        // int shift = (int)(sin(m_LEDTimer.Get()) * kLength / 2 + kLength / 2);

        // Shift at a rate of 10 lights per seconds
        int shift = (int)(m_LEDTimer.Get() * 10);
        // Shift the lights, duplicate at each spacing interval, and wrap the value to the length of the LED array
        int offset = (shift % kLength) % spacing;
        // Final value to determine if an LED should be lit. Separated from the if statement for tidyness
        bool shouldLight = (i % spacing) == offset;
        if (shouldLight)
        {
            m_ledBuffer[i].SetRGB(255, 209, 5);
        }
        else
        {
            m_ledBuffer[i].SetRGB(0, 0, 0); // this is pretty obvious
        }
    }
    updateLEDs();
}

void LEDs::displayOrangeBlue()
{
    double R1 = 252.0;
    double G1 = 215.0;
    double B1 = 0.0;
    double R2 = 115.0;
    double G2 = 169.0;
    double B2 = 255.0;

    for (int i = 0; i < kLength; i++)
    {
        double t = (double)m_LEDTimer.Get() * 1.0;
        double normalizedPosition = (double)i / (double)kLength;

        // math, not an exact science but gets us a smoothly changing pattern between 0 and 1
        double gradientPos = sin(normalizedPosition * M_PI * 4.0 + sin(t * 2) * sin(t * 0.4235523)) * 0.5 + 0.5;

        // Interpolates between the two colors
        m_ledBuffer[i].SetRGB(
            (int)lerp(R1, R2, gradientPos),
            (int)lerp(G1, G2, gradientPos),
            (int)lerp(B1, B2, gradientPos));
    }
    updateLEDs();
}
void LEDs::displayRedWhiteAndBlue()
{

    for (int i = 0; i < kLength; i++)
    {
        double t = (double)m_LEDTimer.Get() * 0.3;
        double normalizedPosition = (double)i / (double)kLength;

        // Modulo magic, keeps it in range 0 - 1, and makes it wrap indefinitely.
        double redStart = std::fmod(t, 1.0);
        double whiteStart = std::fmod(t + 1.0 / 3.0, 1.0);
        double blueStart = std::fmod(t + 2.0 / 3.0, 1.0);

        // Honestly this is a mess.
        // I can try to explain it briefly, but both you AND me would lose brain cells.
        // There is definitely a simpler way, but this is what I went with.

        // So uhm... This first check is just for determining if the LED is within the range for the color in the normal case.
        if (normalizedPosition > redStart && normalizedPosition <= whiteStart)
        {
            m_ledBuffer[i].SetRGB(255, 0, 0);
            // This check handles the case where the start of the next color in the cycle has wrapped around.
        }
        else if (normalizedPosition > redStart && whiteStart < redStart)
        {
            m_ledBuffer[i].SetRGB(255, 0, 0);
            // Repeat for each color.
        }
        else if (normalizedPosition > whiteStart && normalizedPosition <= blueStart)
        {
            m_ledBuffer[i].SetRGB(255, 255, 255);
        }
        else if (normalizedPosition > whiteStart && blueStart < whiteStart)
        {
            m_ledBuffer[i].SetRGB(255, 255, 255);
        }
        else if (normalizedPosition > blueStart && normalizedPosition <= redStart)
        {
            m_ledBuffer[i].SetRGB(0, 0, 255);
        }
        else if (normalizedPosition > blueStart && blueStart < redStart)
        {
            m_ledBuffer[i].SetRGB(0, 0, 255);
        }
    }
    updateLEDs();
}

double LEDs::lerp(double a, double b, double t)
{
    if (a < b)
    {
        return a + t * (b - a);
    } else {
        return b + t * (a - b);
    }
}

void LEDs::displayAutumnColors()
{
    double hue1 = 46.0;
    double sat1 = 255.0;
    double val1 = 255.0;
    double hue2 = 0.0;
    double sat2 = 255.0;
    double val2 = 255.0;

    for (int i = 0; i < kLength; i++)
    {
        double t = (double)m_LEDTimer.Get() * 2.0;
        double normalizedPosition = (double)i / (double)kLength;

        // math, not an exact science but gets us a smoothly changing pattern between 0 and 1
        double gradientPos = sin(normalizedPosition * M_PI * 16 + t) * 0.5 + 0.5;

        // Interpolates between the two colors
        m_ledBuffer[i].SetHSV(
            (int)LEDs::lerp(hue1, hue2, gradientPos),
            (int)LEDs::lerp(sat1, sat2, gradientPos),
            (int)LEDs::lerp(val1, val2, gradientPos));
    }
    updateLEDs();
}
void LEDs::displayWinterColors()
{
    double R1 = 0.0;
    double G1 = 0.0;
    double B1 = 255.0;
    double R2 = 255.0;
    double G2 = 255.0;
    double B2 = 255.0;

    for (int i = 0; i < kLength; i++)
    {
        double t = (double)m_LEDTimer.Get() * 2.0;
        double normalizedPosition = (double)i / (double)kLength;

        // math, not an exact science but gets us a smoothly changing pattern between 0 and 1
        double gradientPos = sin(normalizedPosition * M_PI * 16 + t) * 0.5 + 0.5;

        // Interpolates between the two colors
        m_ledBuffer[i].SetRGB(
            (int)lerp(R1, R2, gradientPos),
            (int)lerp(G1, G2, gradientPos),
            (int)lerp(B1, B2, gradientPos));
    }
    updateLEDs();
}

void LEDs::setRange(int startIndex, int width, char iRed, char iGreen, char iBlue)
{
    if (startIndex < 0 || startIndex > kLength)
        return;
    if (startIndex + width >= kLength)
        width = kLength - startIndex;

    for (int i = startIndex; i < startIndex + width; i++)
    {
        m_ledBuffer[i].SetRGB(iRed, iGreen, iBlue);
    }
}

void LEDs::clear()
{
    for (int i = 0; i < kLength; i++)
    {
        m_ledBuffer[i].SetRGB(0, 0, 0);
    }
}
void LEDs::updateLEDs()
{
    m_led.SetData(m_ledBuffer); // Move the color data from the array to the LED string
}
void LEDs::signalCone()
{
for (int i = 0; i < kLength; i++)
    {
        m_ledBuffer[i].SetRGB(240, 233, 24);
    }
    updateLEDs();
}
void LEDs::signalCube()
{
for (int i = 0; i < kLength; i++)
    {
        m_ledBuffer[i].SetRGB(139, 24, 240);
    }
    updateLEDs();
}
void updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> ldriver = Controllers::instance()->LeftDriver();
    std::shared_ptr<frc::Joystick> rdriver = Controllers::instance()->RightDriver();
    
    bool x = ldriver->GetRawButton(2);
    bool y = rdriver->GetRawButton(2);
    
    if(mode == 't')
    {
     if (x)
     {
        LEDs signalCone();
     }
     else if (y)
     {
        LEDs signalCube();
     }
     else 
     {
        LEDs displayFancyTeamColors();
     }
    }

    if (mode == 'a')
    {
        LEDs displayRainbow();
    }
}
#endif
