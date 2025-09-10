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
import frc.robot.stateMachine.OperatorOptions;

public class LED extends SubsystemBase {
  private final AddressableLED glowbra_commander;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Timer blinkTimer = new Timer();
  private LEDState state = new LEDState(Color.kBlue);
  private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

  public LED() {
    blinkTimer.start();
    glowbra_commander = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(21);
    glowbra_commander.setLength(m_ledBuffer.getLength());
    glowbra_commander.setData(m_ledBuffer);
    glowbra_commander.start();
  }
  @Override
  public void periodic() {

    switch (operatorOptions.scoreLocation) {
      case L1 -> {
        setLED(Color.kBlue);
      }
      case L2 -> {
        setLED(Color.kPurple);
      }
      case L3 -> {
        setLED(Color.kYellow);
      }
      case L4 -> {
        setLED(Color.kGreen);
      }
      case BARGE -> {
        setLED(Color.kRed);
      }
      case PROCESSOR -> {
        setLED(Color.kOrange);
      }
    }
    
    
    if (DriverStation.isDisabled()) {
      LEDPattern.solid(Color.kPurple).applyTo(m_ledBuffer);
    }

    glowbra_commander.setData(m_ledBuffer);
    
  }

  public void setLED(Color color){
    LEDPattern.solid(color).applyTo(m_ledBuffer);
  }

  // public packRGB(){
  //   Color.packRGB(80, 40, 79);
  // }
  
    private static LED instance;

    public static LED getInstance() {
    if (instance == null)
      instance = new LED(); // Make sure there is an instance (this will only run once)
    return instance;
  }
}