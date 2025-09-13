// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lights;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;

import static edu.wpi.first.units.Units.*;

public class LED {
    private final AddressableLED glowjack_horseman;
    private final AddressableLEDBuffer m_ledBuffer;

    //For Climb
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private final Distance ledSpacing = Meters.of(1.0 / 120.0);
    private final LEDPattern scrollingRainbow =
            rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), ledSpacing);
    LEDPattern blinkingRainbow = scrollingRainbow.blink(Seconds.of(.1));
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
        if(Climber.getInstance().getState() != ClimberStates.IDLE){
            if(Climber.getInstance().getState() == ClimberStates.CLIMBING || Climber.getInstance().getState() == ClimberStates.CLIMBED){
                blinkingRainbow.applyTo(m_ledBuffer);
            }else{
                scrollingRainbow.applyTo(m_ledBuffer);
            }
        }else{
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
        }


        glowjack_horseman.setData(m_ledBuffer);

    }
}