package frc.robot.subsystems.armManager.hand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;

public class Hand extends StateMachine<HandStates> {
    private String name = getName();
    private double statorCurrent;
    public static TalonFX motor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();

    private Hand() {
        super(HandStates.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor = new TalonFX(Ports.HandPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
    }

    protected HandStates getNexState(HandStates currentState) {
        return currentState;
    }

    @Override
    public void collectInputs() {
      statorCurrent = motor.getStatorCurrent().getValueAsDouble();
      DogLog.log(name + "/Motor Stator Current", statorCurrent);
    }

    public void setState(HandStates state){
        setStateFromRequest(state);
    }

    public void setHandSpeed(double speed) {
        DogLog.log(getName() + "/Speed", speed);
        motor.set(speed);
    }

    public boolean hasCoral(){
        return statorCurrent > Constants.HandConstants.coralStallCurrent;
    }

    public boolean hasAlgae(){
        return statorCurrent > Constants.HandConstants.algaeStallCurrent;
    }

    @Override
    protected void afterTransition(HandStates newState) {
        switch (newState) {
            case LOLLIPOP -> {
                setHandSpeed(HandSpeeds.LOLLIPOP);
            }
            case IDLE -> {
                setHandSpeed(HandSpeeds.IDLE);
            }
            case SCORE_CORAL -> {
                setHandSpeed(HandSpeeds.SCORE_CORAL);
            }
            case HANDOFF -> {
                setHandSpeed(HandSpeeds.HANDOFF);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                setHandSpeed(HandSpeeds.INTAKE_LOW_REEF_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                setHandSpeed(HandSpeeds.INTAKE_HIGH_REEF_ALGAE);
            }
            case INTAKE_GROUND_ALGAE -> {
                setHandSpeed(HandSpeeds.INTAKE_GROUND_ALAGAE);
            }
            case SCORE_ALGAE_NET -> {
                setHandSpeed(HandSpeeds.SCORE_ALGAE_NET);
            }
            case SCORE_ALGAE_PROCESSOR -> {
                setHandSpeed(HandSpeeds.SCORE_ALGAE_PROCESSOR);
            }
            case CORAL_IDLE -> {
                setHandSpeed(HandSpeeds.CORAL_IDLE);
            }
            case INVERTED_HANDOFF -> {
                setHandSpeed(HandSpeeds.INVERTED_HANDOFF);
            }
        }
    }

    private static Hand instance;

    public static Hand getInstance() {
        if (instance == null)
            instance = new Hand(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
