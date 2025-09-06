// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lights;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.stateMachine.OperatorOptions;

public class LED {
    private final AddressableLED glowjack_horseman;
    private final AddressableLEDBuffer m_ledBuffer;

    public LED() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        glowjack_horseman = new AddressableLED(8);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(150);
        glowjack_horseman.setLength(m_ledBuffer.getLength());
        // Set the data
        glowjack_horseman.setData(m_ledBuffer);
        glowjack_horseman.start();
    }
    public void periodic() {
        switch(OperatorOptions.getInstance().scoreLocation){
            case L1:
                LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
                break;
            case L2:
                LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
                break;
            case L3:
                LEDPattern.solid(Color.kDodgerBlue).applyTo(m_ledBuffer);
                break;
            case L4:
                LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
                break;
            case PROCESSOR:
                LEDPattern.solid(Color.kDarkOliveGreen).applyTo(m_ledBuffer);
                break;
            case BARGE:
                LEDPattern.solid(Color.kPurple).applyTo(m_ledBuffer);
                break;
            default:
                LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer);
                break;
        }
//        if (DriverStation.isDisabled()) {
//            LEDPattern.solid(Color.kPurple).applyTo(m_ledBuffer);
//        } else {
//            LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
//        }

        glowjack_horseman.setData(m_ledBuffer);

    }
}