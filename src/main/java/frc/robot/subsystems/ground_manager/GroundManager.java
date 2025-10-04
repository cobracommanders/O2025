package frc.robot.subsystems.ground_manager;

import com.ctre.phoenix6.Utils;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorState;
import frc.robot.subsystems.ground_manager.intake.IntakePivot;
import frc.robot.subsystems.ground_manager.intake.IntakePivotStates;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollers;
import frc.robot.subsystems.ground_manager.intakeRollers.IntakeRollersStates;

public class GroundManager extends StateMachine<GroundManagerStates> {
    public final String name = getName();

    private final IntakePivot intakePivot = IntakePivot.getInstance();
    private final IntakeRollers rollers = IntakeRollers.getInstance();
    private final CoralDetector coralDetector = CoralDetector.getInstance();

    private GroundManager() {
        super(GroundManagerStates.PREPARE_IDLE);
    }

    @Override
    protected GroundManagerStates getNextState(GroundManagerStates currentState) {
        GroundManagerStates nextState = currentState;
        switch (currentState) {
            case PREPARE_IDLE -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.IDLE;
                }
            }
            case IDLE -> {
            }
            case PREPARE_INTAKE -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.INTAKING;
                }
            }
            case INTAKING -> {
                if (coralDetector.hasCoral()) {
                    nextState = GroundManagerStates.PREPARE_IDLE;
                } else if (Utils.isSimulation() && intakePivot.atGoal() && timeout(Math.random() + 0.5)) {
                    nextState = GroundManagerStates.PREPARE_IDLE;
                    CoralDetectorState simCoralPosition =
                            switch ((int) (Math.random() * 3)) {
                                case 1 -> CoralDetectorState.LEFT;
                                case 2 -> CoralDetectorState.RIGHT;
                                default -> CoralDetectorState.MIDDLE;
                            };
                    coralDetector.setSimCoral(simCoralPosition);
                }
            }
            case PREPARE_HANDOFF -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.WAIT_HANDOFF;
                }
            }
            case HANDOFF -> {
            }
            case WAIT_HANDOFF -> {
            }
            case PREPARE_SCORE_L1 -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.WAIT_SCORE_L1;
                }
            }
            case SCORE_L1 -> {
                if (timeout(1)) {
                    nextState = GroundManagerStates.PREPARE_IDLE;
                } else if (Utils.isSimulation() && intakePivot.atGoal()) {
                    nextState = GroundManagerStates.PREPARE_IDLE;
                    coralDetector.setSimCoral(CoralDetectorState.NONE);
                }
            }
            case WAIT_SCORE_L1 -> {
            }
            case CLIMB -> {
            }
            case PREPARE_INVERTED_HANDOFF -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.WAIT_INVERTED_HANDOFF;
                }
            }
            case WAIT_INVERTED_HANDOFF -> {
            }
        }
        return nextState;
    }

    @Override
    public void afterTransition(GroundManagerStates newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                intakePivot.setState(IntakePivotStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case PREPARE_INTAKE -> {
                intakePivot.setState(IntakePivotStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case PREPARE_HANDOFF -> {
                intakePivot.setState(IntakePivotStates.HANDOFF);

            }
            case PREPARE_SCORE_L1 -> {
                intakePivot.setState(IntakePivotStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case SCORE_L1 -> {
                intakePivot.setState(IntakePivotStates.SCORE_L1);
                rollers.setState(IntakeRollersStates.SCORE_L1);
            }
            case CLIMB -> {
                intakePivot.setState(IntakePivotStates.CLIMB);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case HANDOFF -> {
                rollers.setState(IntakeRollersStates.HANDOFF);
                coralDetector.setSimCoral(CoralDetectorState.NONE);
            }
            case PREPARE_INVERTED_HANDOFF -> {
                intakePivot.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case WAIT_HANDOFF, WAIT_SCORE_L1, IDLE, INTAKING, WAIT_INVERTED_HANDOFF -> {
            }
        }
    }

    public void setState(GroundManagerStates state) {
        setStateFromRequest(state);
    }

    private static GroundManager instance;

    public static GroundManager getInstance() {
        if (instance == null)
            instance = new GroundManager();
        return instance;
    }
}
