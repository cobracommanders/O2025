package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.PhoenixSignalManager;

public class Climber extends StateMachine<ClimberStates> {
  public final DoubleSubscriber climberSpeed = DogLog.tunable("climb/Speed [-1, 1]", 0.0);
  private final DutyCycle encoder;
  private final TalonFX wheelMotor = new TalonFX(Ports.ClimberPorts.WHEEL_CLIMBER_MOTOR_PORT);
  private final TalonFX winchMotor = new TalonFX(Ports.ClimberPorts.WINCH_CLIMBER_MOTOR_PORT);
  private TalonFXConfiguration wheel_motor_config = new TalonFXConfiguration();
  private TalonFXConfiguration winch_motor_config = new TalonFXConfiguration();
  public double climberPosition;
  private MotionMagicVoltage winch_motor_request = new MotionMagicVoltage(0).withSlot(0);
  private double tolerance = 0.05;
  private double motorCurrent = 0.0;
  private double absolutePosition;

  final StatusSignal<Current> motorCurrentSignal = wheelMotor.getStatorCurrent();
  final StatusSignal<Angle> climberPositionSignal = winchMotor.getPosition();

  private Climber() {
    super(ClimberStates.IDLE, "Climber");
    // TODO: update configs
    encoder = new DutyCycle(new DigitalInput(Ports.ClimberPorts.CLIMER_DUTY_CYCLE_ENCODER));
    winch_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wheel_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    winch_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DEPLOY_MOTION_MAGIC_CRUISE_VELOCITY;
    winch_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DEPLOY_MOTION_MAGIC_ACCELERATION;
    winch_motor_config.MotionMagic.MotionMagicJerk = ClimberConstants.DEPLOY_MOTION_MAGIC_JERK;
    climberPosition = 0; // fix this by linking it to absolute encoder

    // wheelMotor.getConfigurator().apply(wheel_motor_config); // TODO These were missing, not sure if intentional
    // winchMotor.getConfigurator().apply(winch_motor_config);

    PhoenixSignalManager.registerSignals(false, motorCurrentSignal, climberPositionSignal);
  }

  @Override
  public void collectInputs() {

    motorCurrent = motorCurrentSignal.getValueAsDouble();
    climberPosition = climberPositionSignal.getValueAsDouble();
    absolutePosition = encoder.getOutput() - ClimberConstants.EncoderOffset;
    // TODO: update climberPosition
    // TODO: log important inputs
    DogLog.log(name + "/Climber postions", absolutePosition);
    DogLog.log(name + "/Cage detection", cageDetected());
    DogLog.log(name + "/Current", motorCurrent);
    // implement input collection (these values will be used in state transitions)
  }

  public void setState(ClimberStates newState) {
    setStateFromRequest(newState);
  }

  public ClimberStates getNextState(ClimberStates currentState) {
    ClimberStates nextState = currentState;
    switch (currentState) {
      case IDLE:
        // none
        break;
        case WAIT_FOR_CAGE:
          if (atGoal()) {
            nextState = ClimberStates.CONTINUE_SUCKING;
          }
          break;
          case CONTINUE_SUCKING:
                  if(timeout(0.5)){
                      nextState = ClimberStates.CLIMBING;
                  }


        case CLIMBING:
           if (atGoal()){
            nextState = ClimberStates.CLIMBED;
           }
           break;
        case CLIMBED:
           //do nothing, robot is done
           break;
      }
      return nextState;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case DEPLOYING -> {
        DogLog.log(name + "/setpoint", ClimberPositions.DEPLOYING);
        yield absolutePosition > ClimberPositions.DEPLOYING;
      }
      case CLIMBING -> {
        DogLog.log(name + "/setpoint", ClimberPositions.CLIMBING);
        yield absolutePosition < ClimberPositions.CLIMBED;
      }
      case WAIT_FOR_CAGE ->
        cageDetected();
      default ->
        false;
    };
  }

  @Override
  protected void afterTransition(ClimberStates currentState) {
    switch (currentState) {
      case IDLE:
        // do nothing
        break;
      case DEPLOYING:
        setWinchSpeed(WinchSpeeds.DEPLOYING);
        break;
      case WAIT_FOR_CAGE:
        // set climber position, set wheels
        setWinchSpeed(WinchSpeeds.IDLE);
        setClimberWheelSpeed(ClimberWheelSpeeds.INTAKE_CAGE);
        break;
      case CLIMBING:
        // set wheels, set climber position
        setWinchSpeed(WinchSpeeds.CLIMBING);
        setClimberWheelSpeed(ClimberWheelSpeeds.INTAKE_CAGE);
        break;
      case CLIMBED:
        setWinchSpeed(ClimberPositions.IDLE);
        // do nothing
        break;
      // case KEEP_SUCKING:
      //   // do nothing
      //   break;
    case CONTINUE_SUCKING:
        //do nothing!
        break;
      default:
        break;
    }
  }

  public void setWinchSpeed(double winchSpeed) {
    winchMotor.set(winchSpeed);
  }

  public void setClimberWheelSpeed(double speed) {
    DogLog.log(name + "/Wheel Speed", speed);
    wheelMotor.set(speed);
  }

  private boolean cageDetected() {
    return motorCurrent > ClimberConstants.CAGE_DETECECTION_CURRENT;
  }

  public static Climber instance;

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

}
