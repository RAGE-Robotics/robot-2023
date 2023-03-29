// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.subsystems.wpilib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ragerobotics.frc2023.commands.LEDs.SignalCone;
import com.ragerobotics.frc2023.commands.LEDs.SignalCube;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  AddressableLED m_led = new AddressableLED(0); 
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(300);

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void allianceColor(){
    if (DriverStation.getAlliance() == Alliance.Red)
    {
      for (var i = 0; i < m_ledBuffer.getLength(); i++)
        {
         m_ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    else
    {
      for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }

    m_led.setData(m_ledBuffer);
  }

  public void signalCone(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 255, 255, 0);
      }

    m_led.setData(m_ledBuffer);
  }

  public void signalCube(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 255, 0, 255);
      }

    m_led.setData(m_ledBuffer);
  }

  public void updateLEDs(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!SignalCube.mCubeActive && !SignalCone.mConeActive)
    {
      allianceColor();
    }
  }
  
}
