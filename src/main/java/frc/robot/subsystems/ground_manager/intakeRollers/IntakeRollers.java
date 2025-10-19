package frc.robot.subsystems.ground_manager.intakeRollers;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import frc.robot.Ports;
import frc.robot.Constants.IntakeRollersConstants;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;


public class IntakeRollers extends StateMachine<IntakeRollersStates>{
  
    private final TalonFX motor;
    private double motorStatorCurrent;

    private CoralDetector coralDetector = CoralDetector.getInstance();
    
    private IntakeRollers() {
        super(IntakeRollersStates.IDLE, "IntakeRollers");
        motor = new TalonFX(Ports.IntakeRollersPorts.INTAKE_ROLLER_MOTOR);
    }
   
    @Override
    public void collectInputs() {
      motorStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
      DogLog.log(name + "/Stator Current", motorStatorCurrent);
    }
    public double getMotorStatorCurrent() {
      return motorStatorCurrent;
  }
  
    public void setState(IntakeRollersStates newState) {
        setStateFromRequest(newState);
    }

    public boolean hasCoral(){
      return coralDetector.hasCoral();
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
