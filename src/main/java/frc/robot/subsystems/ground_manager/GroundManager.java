package frc.robot.subsystems.ground_manager;

import frc.robot.subsystems.ground_manager.intake.Intake;
import frc.robot.subsystems.ground_manager.intake.IntakeStates;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollers;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollersStates;
import frc.robot.subsystems.ground_manager.GroundManagerStates;

import dev.doglog.DogLog;

import java.util.Queue;
import java.util.LinkedList;



public class GroundManager {

    private GroundManagerStates currentState = GroundManagerStates.PREPARE_IDLE;
    private GroundManagerStates nextState = GroundManagerStates.PREPARE_IDLE;

    private final Intake intake;
    private final IntakeRollers rollers;

    private double idlePosition = 0.0;
    private double intakePosition = 0.0;
    private double handoffPosition = 0.0;
    private double scoreL1Position = 0.0;
    private double climbPosition = 0.0;

    public GroundManager() {
        intake = Intake.getInstance();
        rollers = IntakeRollers.getInstance();
    }

    public void periodic() {
        intake.collectInputs();
        rollers.collectInputs();
        
        DogLog.log("GroundManager/State", getStateBooleans());



        getNextState();

        if (nextState != currentState) {
            currentState = nextState;
            afterTransition(currentState);
            DogLog.log("GroundManager/State", getStateBooleans());

        }
    }

    private void getNextState() {
        switch (currentState) {
            case PREPARE_IDLE -> nextState = GroundManagerStates.IDLE;
            case IDLE -> {
                intake.setState(IntakeStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case PREPARE_INTAKE -> {
                intake.setIntakePosition(intakePosition);
                nextState = GroundManagerStates.INTAKING;
            }
            case INTAKING -> {
                intake.setState(IntakeStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case PREPARE_HANDOFF -> {
                intake.setIntakePosition(handoffPosition);
                nextState = GroundManagerStates.WAIT_HANDOFF;
            }
            case WAIT_HANDOFF -> {
                intake.setState(IntakeStates.HANDOFF);
                rollers.setState(IntakeRollersStates.WAIT_HANDOFF);
            }
            case HANDOFF -> {
                intake.setState(IntakeStates.HANDOFF);
                rollers.setState(IntakeRollersStates.HANDOFF);
            }
            case PREPARE_SCORE_L1 -> {
                intake.setIntakePosition(scoreL1Position);
                nextState = GroundManagerStates.WAIT_SCORE_L1;
            }
            case WAIT_SCORE_L1 -> {
                intake.setState(IntakeStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.WAIT_SCORE_L1);
            }
            case SCORE_L1 -> {
                intake.setState(IntakeStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.SCORE_L1);
            }
            case CLIMB -> {
                intake.setState(IntakeStates.CLIMB);
                rollers.setState(IntakeRollersStates.CLIMB);
            }
        }
    }

    private void afterTransition(GroundManagerStates newState) {
        switch (newState) {
            case IDLE -> {
                intake.setState(IntakeStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
                intake.setIntakePosition(idlePosition);
            }
            case INTAKING -> {
                intake.setState(IntakeStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case WAIT_HANDOFF -> {
                intake.setState(IntakeStates.HANDOFF);
                rollers.setState(IntakeRollersStates.WAIT_HANDOFF);
            }
            case HANDOFF -> {
                intake.setState(IntakeStates.HANDOFF);
                rollers.setState(IntakeRollersStates.HANDOFF);
            }
            case WAIT_SCORE_L1 -> {
                intake.setState(IntakeStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.WAIT_SCORE_L1);
            }
            case SCORE_L1 -> {
                intake.setState(IntakeStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.SCORE_L1);
            }
            case CLIMB -> {
                intake.setState(IntakeStates.CLIMB);
                rollers.setState(IntakeRollersStates.CLIMB);
                intake.setIntakePosition(climbPosition);
            }
        }
    }

    public GroundManagerStates getCurrentState() {
        return currentState;
    }

    public void setState(GroundManagerStates newState) {
        nextState = newState;
        DogLog.log("GroundManager/State", getStateBooleans());

    }

    public void setIdlePosition(double pos) { idlePosition = pos; }
    public void setIntakePosition(double pos) { intakePosition = pos; }
    public void setHandoffPosition(double pos) { handoffPosition = pos; }
    public void setScoreL1Position(double pos) { scoreL1Position = pos; }
    public void setClimbPosition(double pos) { climbPosition = pos; }

    public void increaseSetpoint(GroundManagerStates state, double delta) {
        switch (state) {
            case PREPARE_IDLE, IDLE -> idlePosition += delta;
            case PREPARE_INTAKE, INTAKING -> intakePosition += delta;
            case PREPARE_HANDOFF, WAIT_HANDOFF, HANDOFF -> handoffPosition += delta;
            case PREPARE_SCORE_L1, WAIT_SCORE_L1, SCORE_L1 -> scoreL1Position += delta;
            case CLIMB -> climbPosition += delta;
        }
        DogLog.log("GroundManager/IdlePosition", new double[]{idlePosition});
        DogLog.log("GroundManager/IntakePosition", new double[]{intakePosition});
        DogLog.log("GroundManager/HandoffPosition", new double[]{handoffPosition});
        DogLog.log("GroundManager/ScoreL1Position", new double[]{scoreL1Position});
        DogLog.log("GroundManager/ClimbPosition", new double[]{climbPosition});
        

    }

    public void decreaseSetpoint(GroundManagerStates state, double delta) {
        switch (state) {
            case PREPARE_IDLE, IDLE -> idlePosition -= delta;
            case PREPARE_INTAKE, INTAKING -> intakePosition -= delta;
            case PREPARE_HANDOFF, WAIT_HANDOFF, HANDOFF -> handoffPosition -= delta;
            case PREPARE_SCORE_L1, WAIT_SCORE_L1, SCORE_L1 -> scoreL1Position -= delta;
            case CLIMB -> climbPosition -= delta;
        }
        DogLog.log("GroundManager/IdlePosition", new double[]{idlePosition});
        DogLog.log("GroundManager/IntakePosition", new double[]{intakePosition});
        DogLog.log("GroundManager/HandoffPosition", new double[]{handoffPosition});
        DogLog.log("GroundManager/ScoreL1Position", new double[]{scoreL1Position});
        DogLog.log("GroundManager/ClimbPosition", new double[]{climbPosition});

    }
    private boolean[] getStateBooleans() {
        boolean[] states = new boolean[GroundManagerStates.values().length];
        states[currentState.ordinal()] = true;
        return states;
    }
    
}
