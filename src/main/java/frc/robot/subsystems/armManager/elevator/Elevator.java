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
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class Elevator extends StateMachine<ElevatorStates> {
    private final String name = getName();
    public static TalonFX lMotor;
    public static TalonFX rMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ArmConstants.P).withKI(ArmConstants.I)
                    .withKD(ArmConstants.D).withKG(ArmConstants.G)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));

    private double elevatorPosition;
    private final double tolerance;
    private Follower right_motor_request = new Follower(Ports.ElevatorPorts.LMOTOR, true);
    private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Elevator() {
        super(ElevatorStates.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ArmConstants.MotionMagicJerk;

        lMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
        rMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);

        lMotor.getConfigurator().apply(motor_config);
        rMotor.getConfigurator().apply(motor_config);

        tolerance = 0.1;
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

    @Override
    public void periodic() {
        // System.out.println(encoder.get());ph
        super.periodic();
    }

    public void setArmPosition(double position) {
        rMotor.setControl(right_motor_request);
        lMotor.setControl(motor_request.withPosition(position));
    }

    @Override
    protected void afterTransition(ElevatorStates newState) {
        switch (newState) {
            case IDLE -> {
                setArmPosition(ElevatorPositions.IDLE);
            }
            case L4 -> {
                setArmPosition(ElevatorPositions.L4);
            }
            case HIGH_REEF_ALGAE -> {
                setArmPosition(ElevatorPositions.HIGH_REEF_ALGAE);
            }
            case LOW_REEF_ALGAE -> {
                setArmPosition(ElevatorPositions.LOW_REEF_ALGAE);
            }
            case GROUND_ALGAE -> {
                setArmPosition(ElevatorPositions.GROUND_ALGAE);
            }
            case ALGAE_PROCESSOR -> {
                setArmPosition(ElevatorPositions.ALGAE_PROCESSOR);
            }
            case ALGAE_NET -> {
                setArmPosition(ElevatorPositions.ALGAE_NET);
            }
        }
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
