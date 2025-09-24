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
    private final String name = getName();
    private double statorCurrent;
    public static TalonFX motor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();

    public Hand() {
        super(HandState.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor = new TalonFX(Ports.HandPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
    }

    protected HandState getNexState(HandState currentState) {
        return currentState;
    }

    @Override
    public void collectInputs() {
        statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        DogLog.log(name + "/Motor Stator Current", statorCurrent);
    }

    public void setState(HandState state) {
        setStateFromRequest(state);
    }

    public void setHandSpeed(double speed) {
        DogLog.log(getName() + "/Speed", speed);
        motor.set(speed);
    }

    public boolean hasCoral() {
        return statorCurrent > Constants.HandConstants.coralStallCurrent;
    }

    public boolean hasAlgae() {
        return statorCurrent > Constants.HandConstants.algaeStallCurrent;
    }

    @Override
    protected void afterTransition(HandState newState) {
        switch (newState) {
            case LOLLIPOP -> {
                setHandSpeed(HandSpeed.LOLLIPOP);
            }
            case IDLE -> {
                setHandSpeed(HandSpeed.IDLE);
            }
            case SCORE_CORAL -> {
                setHandSpeed(HandSpeed.SCORE_CORAL);
            }
            case HANDOFF -> {
                setHandSpeed(HandSpeed.HANDOFF);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                setHandSpeed(HandSpeed.INTAKE_LOW_REEF_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                setHandSpeed(HandSpeed.INTAKE_HIGH_REEF_ALGAE);
            }
            case INTAKE_GROUND_ALGAE -> {
                setHandSpeed(HandSpeed.INTAKE_GROUND_ALGAE);
            }
            case SCORE_ALGAE_NET -> {
                setHandSpeed(HandSpeed.SCORE_ALGAE_NET);
            }
            case SCORE_ALGAE_PROCESSOR -> {
                setHandSpeed(HandSpeed.SCORE_ALGAE_PROCESSOR);
            }
            case CORAL_IDLE -> {
                setHandSpeed(HandSpeed.CORAL_IDLE);
            }
            case INVERTED_HANDOFF -> {
                setHandSpeed(HandSpeed.INVERTED_HANDOFF);
            }
        }
    }
}
