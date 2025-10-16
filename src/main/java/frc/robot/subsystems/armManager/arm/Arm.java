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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    private static TalonFX motor;
    private final CANcoder encoder;

    private double armPosition;
    private double absolutePosition;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private double customStatePosition = 0.0;

    public Arm() {
        super(ArmState.START_POSITION);
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
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.MotionMagicAcceleration;
        motor_config.ClosedLoopGeneral.ContinuousWrap = true;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.encoderOffset;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.7;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        encoder = new CANcoder(Ports.ArmPorts.ENCODER);
        motor = new TalonFX(Ports.ArmPorts.MOTOR);

        motor.getConfigurator().apply(motor_config);
        encoder.getConfigurator().apply(canCoderConfig);

        collectInputs();
        syncEncoder();
    }

    public boolean atGoal() {
        return MathUtil.isNear(getState().getPosition(), getNormalizedPosition(), ArmConstants.Tolerance);
    }

    public void syncEncoder() {
        if (Utils.isSimulation()) {
            return;
        }
        motor.setPosition(absolutePosition);
    }

    @Override
    protected void collectInputs() {
        absolutePosition = encoder.getPosition().getValueAsDouble();
        armPosition = motor.getPosition().getValueAsDouble();
        DogLog.log("Arm/Absolute Encoder position", absolutePosition);
        DogLog.log("Arm/Motor Encoder Position", armPosition);
        DogLog.log("Arm/Code Position", getNormalizedPosition());
        DogLog.log("Arm/At Goal", atGoal());
        DogLog.log("Arm/CustomPosition", customStatePosition);
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

    public void setState(ArmState state) {
        setStateFromRequest(state);
    }

    public void setCustom(double targetPosition) {
        this.customStatePosition = targetPosition;
        setState(ArmState.CUSTOM);
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
        motor.setControl(motionMagicVoltage.withPosition(position));
    }
}
