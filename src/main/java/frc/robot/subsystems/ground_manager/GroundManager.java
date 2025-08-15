package frc.robot.subsystems.ground_manager;

import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.ground_manager.intake.IntakePivot;
import frc.robot.subsystems.ground_manager.intake.IntakePivotStates;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollers;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollersStates;

public class GroundManager extends StateMachine<GroundManagerStates> {
    public final String name = getName();

    private final IntakePivot intake;
    private final IntakeRollers rollers;

    public GroundManager() {
        super(GroundManagerStates.PREPARE_IDLE);
        intake = IntakePivot.getInstance();
        rollers = IntakeRollers.getInstance();
    }
    @Override
    protected GroundManagerStates getNextState(GroundManagerStates currentState) {
        GroundManagerStates nextState = currentState;
        switch (currentState) {
            case PREPARE_IDLE -> {
                if (intake.atGoal()){
                    nextState = GroundManagerStates.IDLE;
                }
            }
            case IDLE -> {
            }
            case PREPARE_INTAKE -> {
                if (intake.atGoal()){
                    nextState = GroundManagerStates.INTAKING;
                }
             }
            case INTAKING -> {
                if (rollers.hasCoral()){
                    nextState = GroundManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_HANDOFF -> {
                if (intake.atGoal()){
                    nextState = GroundManagerStates.WAIT_HANDOFF;
                }
            }
            case HANDOFF -> {
                //robot manager to handle this transition to have coral in hand
            }
            case WAIT_HANDOFF -> {
            }
            case PREPARE_SCORE_L1 -> {
                if (intake.atGoal()){
                    nextState = GroundManagerStates.WAIT_SCORE_L1;
                }
            }
            case SCORE_L1 -> {
                //add timeout
            }
            case WAIT_SCORE_L1 -> {
            }
            case CLIMB -> {
            }
        }
        return nextState;
    }
@Override
    public void afterTransition(GroundManagerStates newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                intake.setState(IntakePivotStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case PREPARE_INTAKE -> {
                intake.setState(IntakePivotStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case PREPARE_HANDOFF -> {
                intake.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.HANDOFF);
            }
            case PREPARE_SCORE_L1 -> {
                intake.setState(IntakePivotStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case SCORE_L1 -> {
                intake.setState(IntakePivotStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.SCORE_L1);
            }
            case CLIMB -> {
                intake.setState(IntakePivotStates.CLIMB);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case WAIT_HANDOFF, WAIT_SCORE_L1, IDLE, INTAKING, HANDOFF -> {
                
            }
        }
    }
}
