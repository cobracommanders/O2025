// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lights;


import java.security.cert.LDAPCertStoreParameters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.OperatorOptions.CoralMode;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorStates;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LED {
    private final AddressableLED glowjack_horseman;
    private final AddressableLEDBuffer m_ledBuffer;
    private final AddressableLEDBufferView m_middle;

    // Rainbow LED pattern
    public final LEDPattern m_pattern = LEDPattern.rainbow(255, 128);
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);
    private final LEDPattern m_scrollingRainbow =
            m_pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    public LED() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        glowjack_horseman = new AddressableLED(8);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(150);
        glowjack_horseman.setLength(m_ledBuffer.getLength());
        m_middle = m_ledBuffer.createView(8, 12);
        // Set the data
        glowjack_horseman.setData(m_ledBuffer);
        glowjack_horseman.start();
    }
    Color c;
    public void periodic() {
        switch(OperatorOptions.getInstance().scoreLocation){
            case L1:
                c = Color.kRed;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            case L2:
                c = Color.kYellow;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            case L3:
                c = Color.kDodgerBlue;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            case L4:
                c = Color.kGreen;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            case PROCESSOR:
                c = Color.kDarkOliveGreen;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            case BARGE:
                c = Color.kPurple;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
            default:
                c = Color.kBlack;
                LEDPattern.solid(c).applyTo(m_ledBuffer);
                break;
        }

        if(OperatorOptions.getInstance().coralMode == CoralMode.CORAL_MODE){
            LEDPattern.solid(Color.kWhite).applyTo(m_middle);
        }else{
            LEDPattern.solid(c).applyTo(m_middle);
        }

        if(RequestManager.getInstance().climber.getState() != ClimberStates.IDLE){

        }

        glowjack_horseman.setData(m_ledBuffer);

    }
}