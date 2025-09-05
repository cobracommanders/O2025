// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.stateMachine.OperatorOptions;

public class LED extends SubsystemBase {

  AddressableLED glowbra_commander;
  AddressableLEDBuffer m_ledBuffer;

  private final Timer blinkTimer = new Timer();

  private final OperatorOptions operatorOptions;

  public LED(OperatorOptions operatorOptions) {
    this.operatorOptions = operatorOptions;
    blinkTimer.start();
    glowbra_commander = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(21);
    glowbra_commander.setLength(m_ledBuffer.getLength());
    glowbra_commander.setData(m_ledBuffer);
    glowbra_commander.start();
  }

  public void periodic() {

    switch (operatorOptions.scoreLocation) {
      case L1 -> {
        LEDPattern.solid(Color.kAliceBlue);
      }
      case L2 -> {
        LEDPattern.solid(Color.kCoral);
      }
      case L3 -> {
        LEDPattern.solid(Color.kGreen);
      }
      case L4 -> {
        LEDPattern.solid(Color.kYellow);
      }
      case BARGE -> {
        LEDPattern.solid(Color.kRed);
      }
      case PROCESSOR -> {
        LEDPattern.solid(Color.kOrange);
      }
    }

    glowbra_commander.setData(m_ledBuffer);
    
  }
}