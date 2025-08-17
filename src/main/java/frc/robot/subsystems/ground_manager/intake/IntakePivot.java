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
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class IntakePivot extends StateMachine<IntakePivotStates> {
  public final String name = getName();
//TODO update motor configs
  private final TalonFX intakeMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs().withKP(IntakePivotConstants.P).withKI(IntakePivotConstants.I).withKD(IntakePivotConstants.D)
          .withKG(IntakePivotConstants.G).withGravityType(GravityTypeValue.Arm_Cosine))
      .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));
  private double intakePosition;
  private final double tolerance;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

  private IntakePivot() {
    super(IntakePivotStates.IDLE);
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotor = new TalonFX(Ports.IntakePivotPorts.INTAKE_MOTOR);
    intakeMotor.getConfigurator().apply(motor_config);
    motor_config.MotionMagic.MotionMagicCruiseVelocity = IntakePivotConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = IntakePivotConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = IntakePivotConstants.MotionMagicJerk;
    tolerance = 0.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> MathUtil.isNear(IntakePivotPositions.IDLE, intakePosition, tolerance);
      case INTAKING -> MathUtil.isNear(IntakePivotPositions.INTAKING, intakePosition, tolerance);
      case HANDOFF -> MathUtil.isNear(IntakePivotPositions.HANDOFF, intakePosition, tolerance);
      case SCORE_L1 -> MathUtil.isNear(IntakePivotPositions.SCORE_L1, intakePosition, tolerance);
      case CLIMB -> MathUtil.isNear(IntakePivotPositions.CLIMB, intakePosition, tolerance);
      default -> false;
    };
  }

  public void setState(IntakePivotStates newState) {
    setStateFromRequest(newState);
  }

  @Override
  public void collectInputs() {
    intakePosition = intakeMotor.getPosition().getValueAsDouble();
    DogLog.log(name + "/Intake Position", intakePosition);
  }

  public void setIntakePosition(double position) {
    DogLog.log(name + "/Setpoint", position);
    intakeMotor.setControl(motor_request.withPosition(position));
  }

  @Override
  protected void afterTransition(IntakePivotStates newState) {
    switch (newState) {
      case IDLE -> {
        setIntakePosition(IntakePivotPositions.IDLE);
      }
      case INTAKING -> {
        setIntakePosition(IntakePivotPositions.INTAKING);
      }
      case CLIMB -> {
        setIntakePosition(IntakePivotPositions.CLIMB);
      }
      case HANDOFF -> {
        setIntakePosition(IntakePivotPositions.HANDOFF);
      }
      case SCORE_L1 -> {
        setIntakePosition(IntakePivotPositions.SCORE_L1);
      }
    }
  }

  private static IntakePivot instance;

  public static IntakePivot getInstance() {
    if (instance == null)
      instance = new IntakePivot();
    return instance;
  }
}
