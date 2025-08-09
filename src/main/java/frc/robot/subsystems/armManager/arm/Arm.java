package frc.robot.subsystems.armManager.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ArmConstants;
import frc.robot.stateMachine.StateMachine;

public class Arm extends StateMachine<ArmStates> {
    public static TalonFX motor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ArmConstants.P).withKI(ArmConstants.I)
                    .withKD(ArmConstants.D).withKG(ArmConstants.G)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));

    private double armPosition;
    private final double tolerance;
    private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Arm() {
        super(ArmStates.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ArmConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ArmConstants.MotionMagicJerk;

        motor.getConfigurator().apply(motor_config);

        tolerance = 0.1;
    }

    protected ArmStates getNexState(ArmStates currentState) {
        return currentState;
      }
}
