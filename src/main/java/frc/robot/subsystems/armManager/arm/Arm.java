package frc.robot.subsystems.armManager.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.util.PhoenixSignalManager;

public class Arm extends StateMachine<ArmState> {
    private final TalonFX motor = new TalonFX(Ports.ArmPorts.MOTOR, TunerConstants.kCANBus.getName());
    private final CANcoder encoder = new CANcoder(Ports.ArmPorts.ENCODER, TunerConstants.kCANBus.getName());

    private double armPosition;
    private double absolutePosition;

    private double acceleration = ArmConstants.DefaultMotionMagicAcceleration;
    private double overrideAcceleration = Double.NaN;

    private final DynamicMotionMagicVoltage motionMagicVoltage = new DynamicMotionMagicVoltage(0, ArmConstants.DefaultMotionMagicAcceleration, ArmConstants.MotionMagicCruiseVelocity, ArmConstants.MotionMagicJerk).withSlot(0);

    private double customStatePosition = 0.0;

    private final StatusSignal<Angle> absolutePositionSignal = encoder.getPosition();
    private final StatusSignal<Angle> positionSignal = motor.getPosition();

    public Arm() {
        super(ArmState.START_POSITION, "Arm");
        TalonFXConfiguration motor_config = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ArmConstants.P)
                                .withKI(ArmConstants.I)
                                .withKD(ArmConstants.D)
                                .withKG(ArmConstants.G)
                                .withGravityType(GravityTypeValue.Arm_Cosine)
                )
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.ArmGearRatio));
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.DefaultMotionMagicAcceleration;
        motor_config.ClosedLoopGeneral.ContinuousWrap = true;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.encoderOffset;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.7;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        motor.getConfigurator().apply(motor_config);
        encoder.getConfigurator().apply(canCoderConfig);

        PhoenixSignalManager.registerSignals(
                true,
                absolutePositionSignal,
                positionSignal
        );

        collectInputs();
        syncEncoder();
    }

    public boolean atGoal() {
        return MathUtil.isNear(getState().getPosition(), getNormalizedPosition(), ArmConstants.Tolerance, -0.5, 0.5);
    }

    public void syncEncoder() {
        if (Utils.isSimulation()) {
            return;
        }
        motor.setPosition(absolutePosition);
    }

    public void setOverrideAcceleration(double acceleration) {
        this.overrideAcceleration = acceleration;
    }

    public void clearOverrideAcceleration() {
        this.overrideAcceleration = Double.NaN;
    }

    @Override
    protected void collectInputs() {
        absolutePosition = absolutePositionSignal.getValueAsDouble();
        armPosition = positionSignal.getValueAsDouble();

        DogLog.log("Arm/Absolute Encoder position", absolutePosition);
        DogLog.log("Arm/Motor Encoder Position", armPosition);
        DogLog.log("Arm/Normalized Position", getNormalizedPosition());
        DogLog.log("Arm/At Goal", atGoal());
        DogLog.log("Arm/CustomPosition", customStatePosition);
        DogLog.log("Arm/Acceleration", acceleration);
        MechanismVisualizer.setArmPosition(armPosition);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Periodically update
        // afterTransition doesn't work because it is only called once when CUSTOM is set for the first time
        if (getState() == ArmState.CUSTOM) {
            setMotorToTargetPosition(customStatePosition);
        }
    }

    /**
     * Returns the arm position normalized to the range [-0.5, 0.5].
     */
    public double getNormalizedPosition() {
        return normalizePosition(armPosition);
    }

    public static double normalizePosition(double armPosition) {
        double position = armPosition % 1.0;
        if (position > 0.5)
            return position - 1.0;
        if (position < -0.5)
            return position + 1.0;
        return position;
    }

    @Override
    public void simulationPeriodic() {
        SimArm.updateSimPosition(motor, encoder);
    }

    public void setState(ArmState state, double acceleration) {
        this.acceleration = acceleration;
        setStateFromRequest(state);
    }

    public void setCustom(double targetPosition, double acceleration) {
        this.customStatePosition = targetPosition;
        setState(ArmState.CUSTOM, acceleration);
    }

    @Override
    public ArmState getState() {
        return super.getState();
    }

    @Override
    protected void afterTransition(ArmState newState) {
        switch (newState) {
            case CUSTOM -> setMotorToTargetPosition(customStatePosition);


            // Custom cases can go here, default to standard position control
            default -> {
                setMotorToTargetPosition(newState.getPosition());
            }
        }
    }

    private void setMotorToTargetPosition(double position) {
        DogLog.log("Arm/Setpoint", position);
        double finalAcceleration = Double.isNaN(overrideAcceleration) ? acceleration : overrideAcceleration;
        motor.setControl(motionMagicVoltage.withPosition(position).withAcceleration(finalAcceleration));
    }
}
