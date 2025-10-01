// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.config.FeatureFlags;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.subsystems.armManager.ArmManagerState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

import static edu.wpi.first.units.Units.Seconds;

public class LED {
    private final AddressableLED glowjack_horseman;
    private final AddressableLEDBuffer m_ledBuffer;
    private final AddressableLEDBufferView m_middle;

    boolean isBlinking;

    boolean tryCoralBlink = true;

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

        isBlinking = false;
    }

    Color c;

    public void periodic() {
        //when the robot is blinking, the cage has been detected, robot is done for the match
        if (!isBlinking) {
            switch (OperatorOptions.getInstance().scoreLocation) {
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

            if (Robot.armManager.getState().handGamePieceState == ArmManagerState.HandGamePieceState.CORAL) {
                LEDPattern.solid(Color.kWhite).applyTo(m_middle);
            } else {
                LEDPattern.solid(c).applyTo(m_middle);
            }

//            //start blinking the LEDs .5 seconds before the climber starts pulling the robot up
//            if (Climber.getInstance().getState() == ClimberStates.CONTINUE_SUCKING) {
//                LEDPattern.solid(c).blink(Seconds.of(.2)).applyTo(m_ledBuffer);
//                isBlinking = true;
//            }

            //Can we periodically assign the blink pattern? Or does it need to only be assigned once?
            if (FeatureFlags.LED_INTAKE_BLINK.getAsBoolean()) {
                if (CoralDetector.getInstance().hasCoral()) {
                    LEDPattern.solid(c).blink(Seconds.of(.2)).applyTo(m_ledBuffer);
                }

            }

            glowjack_horseman.setData(m_ledBuffer);
        }

    }
}