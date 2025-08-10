package frc.robot.subsystems.ground_manager;

import frc.robot.subsystems.ground_manager.intake.Intake;
import frc.robot.subsystems.ground_manager.intake.IntakeStates;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollers;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollersStates;

public class GroundManager {

    public enum GroundManagerStates {
        PREPARE_IDLE,
        IDLE,
        PREPARE_INTAKE,
        INTAKING,
        PREPARE_HOLDING_CORAL,
        HOLDING_CORAL,
        PREPARE_HANDOFF,
        WAIT_HANDOFF,
        HANDOFF,
        PREPARE_SCORE_L1,
        WAIT_SCORE_L1,
        SCORE_L1,
        CLIMB
    }

    private GroundManagerStates currentState = GroundManagerStates.PREPARE_IDLE;
    private GroundManagerStates nextState = GroundManagerStates.PREPARE_IDLE;

    private final Intake intake;
    private final IntakeRollers rollers;

    private double idlePosition = 0.0;
    private double intakePosition = 0.0;
    private double holdingCoralPosition = 0.0;
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

        calculateNextState();

        if (nextState != currentState) {
            currentState = nextState;
            afterTransition(currentState);
        }
    }

    private void calculateNextState() {
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

            case PREPARE_HOLDING_CORAL -> {
                intake.setIntakePosition(holdingCoralPosition);
                nextState = GroundManagerStates.HOLDING_CORAL;
            }

            case HOLDING_CORAL -> {
                intake.setState(IntakeStates.HOLDING_CORAL);
                rollers.setState(IntakeRollersStates.HOLDING_CORAL);
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
            case PREPARE_IDLE, PREPARE_INTAKE, PREPARE_HOLDING_CORAL, PREPARE_HANDOFF, PREPARE_SCORE_L1 -> {
                
            }
            case IDLE -> {
                intake.setState(IntakeStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
                intake.setIntakePosition(idlePosition);
            }
            case INTAKING -> {
                intake.setState(IntakeStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case HOLDING_CORAL -> {
                intake.setState(IntakeStates.HOLDING_CORAL);
                rollers.setState(IntakeRollersStates.HOLDING_CORAL);
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
    }

    public void setIdlePosition(double pos) { idlePosition = pos; }
    public void setIntakePosition(double pos) { intakePosition = pos; }
    public void setHoldingCoralPosition(double pos) { holdingCoralPosition = pos; }
    public void setHandoffPosition(double pos) { handoffPosition = pos; }
    public void setScoreL1Position(double pos) { scoreL1Position = pos; }
    public void setClimbPosition(double pos) { climbPosition = pos; }

}
