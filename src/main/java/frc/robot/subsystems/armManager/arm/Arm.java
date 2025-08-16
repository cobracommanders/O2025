package frc.robot.subsystems.armManager.arm;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class Arm extends StateMachine<ArmStates> {
    public String name = getName();
    public static TalonFX motor;
    private final CANcoder encoder;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ArmConstants.P).withKI(ArmConstants.I)
                    .withKD(ArmConstants.D).withKG(ArmConstants.G)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));
    private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private double armPosition;
    private final double tolerance;
    private double absolutePosition;
    private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Arm() {
        super(ArmStates.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ArmConstants.MotionMagicJerk;
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.encoderOffset;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        encoder = new CANcoder(Ports.ArmPorts.ENCODER);
        motor = new TalonFX(Ports.ArmPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
        encoder.getConfigurator().apply(canCoderConfig);

        tolerance = 0.1;
    }

    public boolean atGoal() {
        return switch (getState()) {
            case IDLE ->
                MathUtil.isNear(ArmPositions.IDLE, armPosition, tolerance);
            case INTAKE_GROUND_ALGAE ->
                MathUtil.isNear(ArmPositions.INTAKE_GROUND_ALGAE, armPosition, tolerance);
            case INTAKE_HIGH_REEF_ALGAE ->
                MathUtil.isNear(ArmPositions.INTAKE_HIGH_REEF_ALGAE, armPosition, tolerance);
            case INTAKE_LOW_REEF_ALGAE ->
                MathUtil.isNear(ArmPositions.INTAKE_LOW_REEF_ALGAE, armPosition, tolerance);
            case ALGAE_NET ->
                MathUtil.isNear(ArmPositions.ALGAE_NET, armPosition, tolerance);
            case ALGAE_PROCESSOR ->
                MathUtil.isNear(ArmPositions.ALGAE_PROCESSOR, armPosition, tolerance);
            case L4 ->
                MathUtil.isNear(ArmPositions.L4, armPosition, tolerance);
            case SCORE_L4 ->
                MathUtil.isNear(ArmPositions.SCORE_L4, armPosition, tolerance);
            case HANDOFF_LEFT ->
                MathUtil.isNear(ArmPositions.HANDOFF_LEFT, armPosition, tolerance);
            case HANDOFF_MIDDLE ->
                MathUtil.isNear(ArmPositions.HANDOFF_MIDDLE, armPosition, tolerance);
            case HANDOFF_RIGHT ->
                MathUtil.isNear(ArmPositions.HANDOFF_LEFT, armPosition, tolerance);

        };

    }

    public void syncEncoder() {
        motor.setPosition(absolutePosition);
    }

    @Override
    public void collectInputs() {
        absolutePosition = encoder.getPosition().getValueAsDouble();
        DogLog.log(getName() + "/Current Position", absolutePosition);
    }

    public void setState(ArmStates state) {
        setStateFromRequest(state);
    }

    public void setArmPosition(double position) {
        DogLog.log(name + "/Setpoint", position);
        motor.setControl(motor_request.withPosition(position));
    }

    @Override
    protected void afterTransition(ArmStates newState) {
        switch (newState) {
            case IDLE -> {
                setArmPosition(ArmPositions.IDLE);
            }
            case INTAKE_GROUND_ALGAE -> {
                setArmPosition(ArmPositions.INTAKE_GROUND_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                setArmPosition(ArmPositions.INTAKE_HIGH_REEF_ALGAE);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                setArmPosition(ArmPositions.INTAKE_LOW_REEF_ALGAE);
            }
            case ALGAE_PROCESSOR -> {
                setArmPosition(ArmPositions.ALGAE_PROCESSOR);
            }
            case ALGAE_NET -> {
                setArmPosition(ArmPositions.ALGAE_NET);
            }
            case L4 -> {
                setArmPosition(ArmPositions.L4);
            }
            case SCORE_L4 -> {
                setArmPosition(ArmPositions.SCORE_L4);
            }
            case HANDOFF_LEFT -> {
                setArmPosition(ArmPositions.HANDOFF_LEFT);
            }
            case HANDOFF_MIDDLE -> {
                setArmPosition(ArmPositions.HANDOFF_MIDDLE);
            }
            case HANDOFF_RIGHT -> {
                setArmPosition(ArmPositions.HANDOFF_RIGHT);
            }
        }
    }

    private static Arm instance;

    public static Arm getInstance() {
        if (instance == null)
            instance = new Arm(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
