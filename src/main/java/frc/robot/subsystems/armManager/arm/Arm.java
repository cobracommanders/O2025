package frc.robot.subsystems.armManager.arm;

import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    public String name = getName();

    public final DoubleSubscriber armSpeed = DogLog.tunable("Arm/Speed [-1, 1]", 0.0);
    public static TalonFX motor;
    private final CANcoder encoder;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ArmConstants.P).withKI(ArmConstants.I)
                    .withKD(ArmConstants.D).withKG(ArmConstants.G)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.ArmGearRatio));
    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private double armPosition;
    private final double tolerance;
    private double absolutePosition;
    private final MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Arm() {
        super(ArmState.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ArmConstants.MotionMagicJerk;
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.encoderOffset;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.7;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        encoder = new CANcoder(Ports.ArmPorts.ENCODER);
        motor = new TalonFX(Ports.ArmPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
        encoder.getConfigurator().apply(canCoderConfig);

        collectInputs();
        syncEncoder();

        tolerance = 0.005;
    }

    public boolean atGoal() {
        return switch (getState()) {
            case LOLLIPOP -> MathUtil.isNear(ArmPosition.LOLLIPOP, armPosition, tolerance);
            case IDLE -> MathUtil.isNear(ArmPosition.IDLE, armPosition, tolerance);
            case INTAKE_GROUND_ALGAE -> MathUtil.isNear(ArmPosition.INTAKE_GROUND_ALGAE, armPosition, tolerance);
            case INTAKE_HIGH_REEF_ALGAE ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.INTAKE_HIGH_REEF_ALGAE) : ArmPosition.INTAKE_HIGH_REEF_ALGAE, armPosition, tolerance);
            case INTAKE_LOW_REEF_ALGAE ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.INTAKE_LOW_REEF_ALGAE) : ArmPosition.INTAKE_LOW_REEF_ALGAE, armPosition, tolerance);
            case ALGAE_NET ->
                    MathUtil.isNear(invertNet() ? invertPosition(ArmPosition.ALGAE_NET) : ArmPosition.ALGAE_NET, armPosition, tolerance);
            case ALGAE_PROCESSOR -> MathUtil.isNear(ArmPosition.ALGAE_PROCESSOR, armPosition, tolerance);
            case L4 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.L4) : ArmPosition.L4, armPosition, tolerance);
            case SCORE_L4 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.SCORE_L4) : ArmPosition.SCORE_L4, armPosition, tolerance);
            case L3 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.L3) : ArmPosition.L3, armPosition, tolerance);
            case SCORE_L3 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.SCORE_L3) : ArmPosition.SCORE_L3, armPosition, tolerance);
            case L2 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.L2) : ArmPosition.L2, armPosition, tolerance);
            case SCORE_L2 ->
                    MathUtil.isNear(invertArm() ? invertPosition(ArmPosition.SCORE_L2) : ArmPosition.SCORE_L2, armPosition, tolerance);
            case HANDOFF_LEFT -> MathUtil.isNear(ArmPosition.HANDOFF_LEFT, armPosition, tolerance);
            case HANDOFF_MIDDLE -> MathUtil.isNear(ArmPosition.HANDOFF_MIDDLE, armPosition, tolerance);
            case HANDOFF_RIGHT -> MathUtil.isNear(ArmPosition.HANDOFF_RIGHT, armPosition, tolerance);
            case CLIMB -> MathUtil.isNear(ArmPosition.CLIMB, armPosition, tolerance);
        };

    }

    public void syncEncoder() {
        if (Utils.isSimulation()) {
            return;
        }
        motor.setPosition(absolutePosition);
    }

    public boolean invertArm() {
        switch (AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose2d(), true, true)) {
            case LEFT:
                return true;
            case RIGHT:
                return false;
            default:
                return false;
        }
    }

    public double invertPosition(double position) {
        return -position + 0.5;
    }

    public boolean invertNet() {
        switch (AutoAlign.getNetScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose2d())) {
            case LEFT:
                return true;
            case RIGHT:
                return false;
            default:
                return false;
        }
    }


    @Override
    public void collectInputs() {
        absolutePosition = encoder.getPosition().getValueAsDouble();
        armPosition = motor.getPosition().getValueAsDouble();
        DogLog.log(getName() + "/Encoder position", absolutePosition);
        DogLog.log(getName() + "/Motor position", armPosition);
        DogLog.log(getName() + "/at goal", atGoal());
        if (Utils.isSimulation())
            SimArm.updateSimPosition(motor, encoder);
        // updateSimPosition(setpoint);
        MechanismVisualizer.setArmPosition(armPosition);
    }

    //   @Override
    // public void simulationPeriodic() {
    //     SimArm.updateSimPosition(motor, encoder);
    // }

    @Override
    public void periodic() {
        super.periodic();
        switch (getState()) {
            case L2:
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L2) : ArmPosition.L2);
                break;
            case L3:
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L3) : ArmPosition.L3);
                break;
            case L4:
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L4) : ArmPosition.L4);
                break;
            case INTAKE_HIGH_REEF_ALGAE:
                setArmPosition(invertArm() ? invertPosition(ArmPosition.INTAKE_HIGH_REEF_ALGAE) : ArmPosition.INTAKE_HIGH_REEF_ALGAE);
                break;
            case INTAKE_LOW_REEF_ALGAE:
                setArmPosition(invertArm() ? invertPosition(ArmPosition.INTAKE_LOW_REEF_ALGAE) : ArmPosition.INTAKE_LOW_REEF_ALGAE);
                break;
            case ALGAE_NET:
                setArmPosition(invertNet() ? invertPosition(ArmPosition.ALGAE_NET) : ArmPosition.ALGAE_NET);
                break;

            default:
                break;
        }
    }

    public void setArmSpeed() {
        motor.set(armSpeed.get());
    }

    public void setState(ArmState state) {
        setStateFromRequest(state);
    }

    public void setArmPosition(double position) {
        DogLog.log(name + "/Setpoint", position);
        motor.setControl(motor_request.withPosition(position));
    }

    @Override
    protected void afterTransition(ArmState newState) {
        switch (newState) {
            case LOLLIPOP -> {
                setArmPosition(ArmPosition.LOLLIPOP);
            }
            case IDLE -> {
                setArmPosition(ArmPosition.IDLE);
            }
            case INTAKE_GROUND_ALGAE -> {
                setArmPosition(ArmPosition.INTAKE_GROUND_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.INTAKE_HIGH_REEF_ALGAE) : ArmPosition.INTAKE_HIGH_REEF_ALGAE);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.INTAKE_LOW_REEF_ALGAE) : ArmPosition.INTAKE_LOW_REEF_ALGAE);
            }
            case ALGAE_PROCESSOR -> {
                setArmPosition(ArmPosition.ALGAE_PROCESSOR);
            }
            case ALGAE_NET -> {
                setArmPosition(invertNet() ? invertPosition(ArmPosition.ALGAE_NET) : ArmPosition.ALGAE_NET);
            }
            case L4 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L4) : ArmPosition.L4);
            }
            case SCORE_L4 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.SCORE_L4) : ArmPosition.SCORE_L4);
            }
            case HANDOFF_LEFT -> {
                setArmPosition(ArmPosition.HANDOFF_LEFT);
            }
            case HANDOFF_MIDDLE -> {
                setArmPosition(ArmPosition.HANDOFF_MIDDLE);
            }
            case HANDOFF_RIGHT -> {
                setArmPosition(ArmPosition.HANDOFF_RIGHT);
            }
            case L3 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L3) : ArmPosition.L3);
            }
            case SCORE_L3 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.SCORE_L3) : ArmPosition.SCORE_L3);
            }
            case L2 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.L2) : ArmPosition.L2);
            }
            case SCORE_L2 -> {
                setArmPosition(invertArm() ? invertPosition(ArmPosition.SCORE_L2) : ArmPosition.SCORE_L2);
            }
            case CLIMB -> {
                setArmPosition(ArmPosition.CLIMB);
            }
        }
    }
}
