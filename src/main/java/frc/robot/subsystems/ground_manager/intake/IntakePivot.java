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
import dev.doglog.internal.tunable.Tunable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Ports;
import frc.robot.TunablePid;
import frc.robot.stateMachine.StateMachine;

public class IntakePivot extends StateMachine<IntakePivotStates> {
  public final String name = getName();

  // Create a tunable double with a key of "Intake/Voltage" and a default value of
  // 4.5
  public final DoubleSubscriber speed = DogLog.tunable("Intake/Speed [-1, 1]", 0.0);

  // Create a tunable double with a key of "Intake/Voltage" and a default value of
  // 4.5
  // TODO update motor configs
  private final DutyCycle encoder;
  public final TalonFX intakeMotor;
  public final TalonFXConfiguration motor_config = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs().withKP(IntakePivotConstants.P).withKI(IntakePivotConstants.I)
          .withKD(IntakePivotConstants.D)
          .withKG(IntakePivotConstants.G).withGravityType(GravityTypeValue.Arm_Cosine))// the intake doesn't need to use
                                                                                       // the cosine function to stay up
      .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(IntakePivotConstants.PivotGearRatio));
  private double intakePosition;
  private final double tolerance;
  private double absolutePosition;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

  private IntakePivot() {
    super(IntakePivotStates.IDLE);
    encoder = new DutyCycle(new DigitalInput(Ports.IntakePivotPorts.INTAKE_PIVOT_DUTY_CYCLE_ENCODER));
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotor = new TalonFX(Ports.IntakePivotPorts.INTAKE_MOTOR);
    motor_config.MotionMagic.MotionMagicCruiseVelocity = IntakePivotConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = IntakePivotConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = IntakePivotConstants.MotionMagicJerk;
    intakeMotor.getConfigurator().apply(motor_config);
    tolerance = 0.0;
    collectInputs();
    syncEncoder();
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
    absolutePosition = 1 - encoder.getOutput() - 0.163;
    DogLog.log(name + "/motor Position", intakePosition);
    DogLog.log(name + "/absolute Position", absolutePosition);
  }

  public void setIntakePosition(double position) {
    DogLog.log(name + "/Setpoint", position);
    intakeMotor.setControl(motor_request.withPosition(position));
  }

  public void setIntakeSpeed() {
    intakeMotor.set(speed.get());
  }

  public void syncEncoder() {
    intakeMotor.setPosition(absolutePosition);
  }

  // public void setIntakeKP() {
  // TunablePid.create("intake", intakeMotor, new TalonFXConfiguration());
  // }

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
