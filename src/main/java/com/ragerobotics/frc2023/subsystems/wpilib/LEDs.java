// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.subsystems.wpilib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  AddressableLED m_led = new AddressableLED(9); 
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(50);

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());
  }

  public void redAlliance(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }

    m_led.setData(m_ledBuffer);
  }

  public void blueAlliance(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }

    m_led.setData(m_ledBuffer);
  }

  public void signalCone(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 255, 200, 55);
      }

    m_led.setData(m_ledBuffer);
  }

  public void signalCube(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 175, 55, 255);
      }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
