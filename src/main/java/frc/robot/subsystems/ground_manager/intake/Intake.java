package frc.robot.subsystems.ground_manager.intake;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class Intake extends StateMachine<IntakeStates> {
  public final String name = getName();
//TODO update motor configs
  private final TalonFX intakeMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs().withKP(IntakeConstants.P).withKI(IntakeConstants.I).withKD(IntakeConstants.D)
          .withKG(IntakeConstants.G).withGravityType(GravityTypeValue.Arm_Cosine))
      .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));
  private double intakePosition;
  private final double tolerance;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

  private Intake() {
    super(IntakeStates.IDLE);
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotor = new TalonFX(Ports.IntakePorts.INTAKE_MOTOR);
    intakeMotor.getConfigurator().apply(motor_config);
    motor_config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = IntakeConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = IntakeConstants.MotionMagicJerk;
    tolerance = 0.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> MathUtil.isNear(IntakePositions.IDLE, intakePosition, tolerance);
      case INTAKING -> MathUtil.isNear(IntakePositions.INTAKING, intakePosition, tolerance);
      case HANDOFF -> MathUtil.isNear(IntakePositions.HANDOFF, intakePosition, tolerance);
      case SCORE_L1 -> MathUtil.isNear(IntakePositions.SCORE_L1, intakePosition, tolerance);
      case CLIMB -> MathUtil.isNear(IntakePositions.CLIMB, intakePosition, tolerance);
      default -> false;
    };
  }

  public void setState(IntakeStates newState) {
    setStateFromRequest(newState);
  }

  @Override
  public void collectInputs() {
    intakePosition = intakeMotor.getPosition().getValueAsDouble();
    DogLog.log(name + "/Intake Position", intakePosition);
  }

  public void setIntakePosition(double position) {
    intakeMotor.setControl(motor_request.withPosition(position));
  }

  @Override
  protected void afterTransition(IntakeStates newState) {
    switch (newState) {
      case IDLE -> {
        setIntakePosition(IntakePositions.IDLE);
      }
      case INTAKING -> {
        setIntakePosition(IntakePositions.INTAKING);
      }
      case CLIMB -> {
        setIntakePosition(IntakePositions.CLIMB);
      }
      case HANDOFF -> {
        setIntakePosition(IntakePositions.HANDOFF);
      }
      case SCORE_L1 -> {
        setIntakePosition(IntakePositions.SCORE_L1);
      }
    }
  }

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }
}
