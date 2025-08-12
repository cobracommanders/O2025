package frc.robot.subsystems.ground_manager.intakeRollers;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Ports;
import frc.robot.Constants.IntakeRollersConstants;
import frc.robot.stateMachine.StateMachine;


public class IntakeRollers extends StateMachine<IntakeRollersStates>{
    private final TalonFX motor;
    private double motorStatorCurrent;
    
    public IntakeRollers() {
        super(IntakeRollersStates.IDLE);
        motor = new TalonFX(Ports.IntakeRollersPorts.INTAKE_ROLLER_MOTOR);
    }

    @Override
    public void collectInputs() {
      motorStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
    }
    public double getMotorStatorCurrent() {
      return motorStatorCurrent;
  }
  
    public void setState(IntakeRollersStates newState) {
        setStateFromRequest(newState);
      }

      public boolean hasCoral(){
        if (motorStatorCurrent > IntakeRollersConstants.stallCurrent){
          return true;
        } else {
          return false;
        }
      }

    public void setIntakeRollerSpeeds(double IntakeRollersSpeeds){
      motor.set(IntakeRollersSpeeds);
    }

      @Override
      protected void afterTransition(IntakeRollersStates newState) {
        switch (newState) {
          case IDLE -> {
            setIntakeRollerSpeeds(IntakeRollersSpeeds.IDLE);
          }
          case INTAKING -> {
            setIntakeRollerSpeeds(IntakeRollersSpeeds.INTAKING);
          }
          case HANDOFF -> {
            setIntakeRollerSpeeds(IntakeRollersSpeeds.HANDOFF);
          }
          case SCORE_L1 -> {
            setIntakeRollerSpeeds(IntakeRollersSpeeds.SCORE_L1);
          }
          default -> {}
        }
      }

    private static IntakeRollers instance;

    public static IntakeRollers getInstance() {
        if (instance == null) instance = new IntakeRollers(); 
        return instance;
    }
}
