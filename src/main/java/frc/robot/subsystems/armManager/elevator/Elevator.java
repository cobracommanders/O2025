package frc.robot.subsystems.armManager.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.ArmStates;

public class Elevator extends StateMachine<ElevatorStates> {
    public final DoubleSubscriber elevatorSpeed = DogLog.tunable("elevator/Speed [-1, 1]", 0.0);
    private final String name = getName();
    public static TalonFX lMotor;
    public static TalonFX rMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I)
                    .withKD(ElevatorConstants.D).withKG(ElevatorConstants.G)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.ElevatorGearRatio));

    private double elevatorPosition;
    private final double tolerance;
    private Follower right_motor_request = new Follower(Ports.ElevatorPorts.LMOTOR, true);
    private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Elevator() {
        super(ElevatorStates.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;

        lMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
        rMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);

        lMotor.getConfigurator().apply(motor_config);
        rMotor.getConfigurator().apply(motor_config);

        lMotor.setPosition(0);
        rMotor.setPosition(0);

        tolerance = 0.01;
    }

    protected ElevatorStates getNexState(ElevatorStates currentState) {
        return currentState;
    }

    public boolean atGoal() {
        return switch (getState()) {
            case IDLE ->
                MathUtil.isNear(ElevatorPositions.IDLE, elevatorPosition, tolerance);
            case L4 ->
                MathUtil.isNear(ElevatorPositions.L4, elevatorPosition, tolerance);
            case HIGH_REEF_ALGAE ->
                MathUtil.isNear(ElevatorPositions.HIGH_REEF_ALGAE, elevatorPosition, tolerance);
            case LOW_REEF_ALGAE ->
                MathUtil.isNear(ElevatorPositions.LOW_REEF_ALGAE, elevatorPosition, tolerance);
            case GROUND_ALGAE ->
                MathUtil.isNear(ElevatorPositions.GROUND_ALGAE, elevatorPosition, tolerance);
            case ALGAE_NET ->
                MathUtil.isNear(ElevatorPositions.ALGAE_NET, elevatorPosition, tolerance);
            case ALGAE_PROCESSOR ->
                MathUtil.isNear(ElevatorPositions.ALGAE_PROCESSOR, elevatorPosition, tolerance);
            case HANDOFF ->
                MathUtil.isNear(ElevatorPositions.HANDOFF, elevatorPosition, tolerance);
            case SCORE_L4 ->
                MathUtil.isNear(ElevatorPositions.SCORE_L4, elevatorPosition, tolerance);
        };

    }

    @Override
    public void collectInputs() {
        elevatorPosition = lMotor.getPosition().getValueAsDouble();
        double leftElevatorPosition = lMotor.getPosition().getValueAsDouble();
        double rightElevatorPosition = rMotor.getPosition().getValueAsDouble();
        DogLog.log(name + "/Left Elevator Position", leftElevatorPosition);
        DogLog.log(name + "/Right Elevator Position", rightElevatorPosition);
    }

    public void setState(ElevatorStates state){
        setStateFromRequest(state);
    }

    public void setElevatorSpeed() {
        lMotor.set(elevatorSpeed.get());
        rMotor.set(-elevatorSpeed.get());
      }

    public void setElevatorPosition(double position) {
        DogLog.log(name + "/Setpoint", position);
        rMotor.setControl(right_motor_request);
        lMotor.setControl(motor_request.withPosition(position));
    }

    @Override
    protected void afterTransition(ElevatorStates newState) {
        switch (newState) {
            case IDLE -> {
                setElevatorPosition(ElevatorPositions.IDLE);
            }
            case L4 -> {
                setElevatorPosition(ElevatorPositions.L4);
            }
            case HIGH_REEF_ALGAE -> {
                setElevatorPosition(ElevatorPositions.HIGH_REEF_ALGAE);
            }
            case LOW_REEF_ALGAE -> {
                setElevatorPosition(ElevatorPositions.LOW_REEF_ALGAE);
            }
            case GROUND_ALGAE -> {
                setElevatorPosition(ElevatorPositions.GROUND_ALGAE);
            }
            case ALGAE_PROCESSOR -> {
                setElevatorPosition(ElevatorPositions.ALGAE_PROCESSOR);
            }
            case ALGAE_NET -> {
                setElevatorPosition(ElevatorPositions.ALGAE_NET);
            }
            case HANDOFF -> {
                setElevatorPosition(ElevatorPositions.HANDOFF);
            }
            case SCORE_L4 -> {
                setElevatorPosition(ElevatorPositions.SCORE_L4);
            }
        }
    }

    public void tickUp(){
        switch(getState()){
            case L4:
                ElevatorPositions.L4 += .015;
                setElevatorPosition(ElevatorPositions.L4);
            break;

        }
    }

    public void tickDown(){
        switch(getState()){
            case L4:
                ElevatorPositions.L4 -= .015;
                setElevatorPosition(ElevatorPositions.L4);
            break;

        }
    }
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
