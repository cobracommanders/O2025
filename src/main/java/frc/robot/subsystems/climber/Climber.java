package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class Climber extends StateMachine<ClimberStates>{
    private final String name = getName();
    private final TalonFX wheelMotor;
    private final TalonFX winchMotor;
    private TalonFXConfiguration wheel_motor_config = new TalonFXConfiguration();
    private TalonFXConfiguration winch_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0).withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
    public double climberPosition;
    private MotionMagicVoltage winch_motor_request = new MotionMagicVoltage(0).withSlot(0);
    private double tolerance = 0.0;
    private double motorCurrent = 0.0;
    public Climber(){
      super(ClimberStates.IDLE);
      //TODO: update configs
      winch_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      wheel_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      winch_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DEPLOY_MOTION_MAGIC_CRUISE_VELOCITY;
      winch_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DEPLOY_MOTION_MAGIC_ACCELERATION;
      winch_motor_config.MotionMagic.MotionMagicJerk = ClimberConstants.DEPLOY_MOTION_MAGIC_JERK;
      winchMotor = new TalonFX(Ports.Climber.WINCH_CLIMBER_MOTOR_PORT);
      wheelMotor = new TalonFX(Ports.Climber.WHEEL_CLIMBER_MOTOR_PORT);
      climberPosition = 0; //fix this by linking it to absolute encoder
    }
   
  
  @Override
  public void collectInputs(){
    motorCurrent = wheelMotor.getStatorCurrent().getValueAsDouble();
    climberPosition = winchMotor.getPosition().getValueAsDouble();
    //TODO: update climberPosition
    //TODO: log important inputs
    DogLog.log(name + "/ climber postions", climberPosition);
    DogLog.log(name + "/ cage detection", cageDetected());
    //implement input collection (these values will be used in state transitions)
  }


  public void setState(ClimberStates newState) {
    setStateFromRequest(newState);
  }

  public ClimberStates getNextState(ClimberStates currentState){
      ClimberStates nextState = currentState;
      switch(currentState){
        case IDLE:
          //none
          break;
        case DEPLOYED:
          if (cageDetected()) {
            nextState = ClimberStates.CLIMBING;
          }
          break;
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

  public boolean atGoal(){
    ClimberStates currentState = getState();
    switch(currentState){
      case IDLE:
        return MathUtil.isNear(ClimberPositions.IDLE, climberPosition, tolerance);
      case DEPLOYED:
        return MathUtil.isNear(ClimberPositions.DEPLOYED, climberPosition, tolerance);
      case CLIMBING:
        return MathUtil.isNear(ClimberPositions.DEPLOYED, climberPosition, tolerance);
      case CLIMBED:
        return MathUtil.isNear(ClimberPositions.CLIMBED, climberPosition, tolerance);
      default:
        return false;
    }
  }

  @Override
  protected void afterTransition(ClimberStates currentState){
    switch(currentState){
      case IDLE:
        //do nothing
        break;
      case DEPLOYED:
        //set climber position, set wheels
        setClimberPosition(ClimberPositions.DEPLOYED);
        setClimberWheelSpeed(ClimberWheelSpeeds.DEPLOYED);
        break;
      case CLIMBING:
        //set wheels, set climber position
        setClimberPosition(ClimberPositions.CLIMBING);
        setClimberWheelSpeed(ClimberWheelSpeeds.CLIMBING);
        break;
      case CLIMBED:
        //do nothing
        break;
      default:
        break;
    }
  }

  public void setClimberPosition(double climberSetpoint){
    winchMotor.setControl(winch_motor_request.withPosition(climberSetpoint));
  }
   public void setClimberWheelSpeed(double speed){
    wheelMotor.set(speed);
   }

  private boolean cageDetected(){
    return motorCurrent >  ClimberConstants.CAGE_DETECECTION_CURRENT;
  }

  public Climber instance;
  public Climber getInstance(){
    if(instance == null){
      instance = new Climber();
    }
    return instance;
  }

}
