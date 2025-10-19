package frc.robot.subsystems.armManager.hand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class Hand extends StateMachine<HandState> {
    public static TalonFX motor;

    private double statorCurrent;

    public Hand() {
        super(HandState.IDLE_CORAL, "Hand");
        TalonFXConfiguration motor_config = new TalonFXConfiguration();
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor = new TalonFX(Ports.HandPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
    }

    @Override
    public void collectInputs() {
        statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        DogLog.log("Hand/Stator Current", statorCurrent);
    }

    public void setState(HandState state) {
        setStateFromRequest(state);
    }

    public boolean hasAlgaeForIntake() {
        return statorCurrent > Constants.HandConstants.intakeAlgaeStallCurrent;
    }

    public boolean droppedAlgae() {
        return statorCurrent > Constants.HandConstants.hasAlgaeStallCurrent;
    }

    @Override
    protected void afterTransition(HandState newState) {
        // Custom cases can go here, default to regular speed control
        switch (newState) {
            default -> {
                double speed = newState.getSpeed();
                DogLog.log("Hand/Speed", speed);
                motor.set(speed);
            }
        }
    }
}
