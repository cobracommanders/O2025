package frc.robot.subsystems.ground_manager;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            case PREPARE_INTAKE -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.INTAKING;
                }
            }
            case INTAKING -> {
                if (coralDetector.getState() == CoralDetectorState.MIDDLE) {
                    nextState = GroundManagerStates.PREPARE_IDLE;
                } else if (Utils.isSimulation()
                        && intakePivot.atGoal()
                        && timeout(Math.random() + 0.5)) {
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
                    nextState = GroundManagerStates.READY_HANDOFF;
                }
            }
            case PREPARE_SCORE_L1 -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.READY_SCORE_L1;
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
            case PREPARE_INVERTED_HANDOFF -> {
                if (intakePivot.atGoal()) {
                    nextState = GroundManagerStates.READY_INVERTED_HANDOFF;
                }
            }
            case IDLE, READY_SCORE_L1, CLIMB, READY_INVERTED_HANDOFF, READY_HANDOFF, HANDOFF -> {
                /* Await Control */
            }
        }
        return nextState;
    }

    @Override
    public void afterTransition(GroundManagerStates newState) {
        switch (newState) {
            case PREPARE_IDLE, IDLE -> {
                intakePivot.setState(IntakePivotStates.IDLE);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case PREPARE_INTAKE, INTAKING -> {
                intakePivot.setState(IntakePivotStates.INTAKING);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case PREPARE_HANDOFF, READY_HANDOFF -> {
                intakePivot.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.IDLE);
            }
            case PREPARE_SCORE_L1, READY_SCORE_L1 -> {
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
                intakePivot.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.HANDOFF);

                coralDetector.setSimCoral(CoralDetectorState.NONE);
            }
            case INVERTED_HANDOFF -> {
                intakePivot.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.INTAKING);
            }
            case PREPARE_INVERTED_HANDOFF, READY_INVERTED_HANDOFF -> {
                intakePivot.setState(IntakePivotStates.HANDOFF);
                rollers.setState(IntakeRollersStates.IDLE);
            }
        }
    }

    public void requestIdle() {
        setStateFromRequest(GroundManagerStates.IDLE);
    }

    public void requestHandoffExecution() {
        setStateFromRequest(GroundManagerStates.HANDOFF);
    }

    public void requestInvertedHandoffExecution() {
        setStateFromRequest(GroundManagerStates.INVERTED_HANDOFF);
    }

    public static class CommandWrapper {
        private final GroundManager groundManager;

        public CommandWrapper(GroundManager groundManager) {
            this.groundManager = groundManager;
        }

        private Command setState(GroundManagerStates state) {
            return groundManager
                    .runOnce(() -> groundManager.setStateFromRequest(state))
                    .withName("setState/" + state.name());
        }

        public Command idleAndAwaitReady() {
            return groundManager
                    .runOnce(groundManager::requestIdle)
                    .andThen(groundManager.waitForState(GroundManagerStates.IDLE))
                    .withName("idleAndAwaitReady");
        }

        public Command climbAndDoNothing() {
            return setState(GroundManagerStates.CLIMB)
                    .andThen(doNothing())
                    .withName("climbAndDoNothing");
        }

        /** Prepare for handoff and await ready state. */
        public Command requestHandoffAndAwaitReady() {
            return setState(GroundManagerStates.PREPARE_HANDOFF)
                    .andThen(groundManager.waitForState(GroundManagerStates.READY_HANDOFF))
                    .withName("requestHandoff");
        }

        /** Prepare for inverted handoff and await ready state. */
        public Command requestInvertedHandoffAndAwaitReady() {
            return setState(GroundManagerStates.PREPARE_INVERTED_HANDOFF)
                    .andThen(groundManager.waitForState(GroundManagerStates.READY_INVERTED_HANDOFF))
                    .withName("requestInvertedHandoff");
        }

        /** Intake until a piece is detected. */
        public Command intakeUntilPiece() {
            return setState(GroundManagerStates.INTAKING)
                    .andThen(awaitGamePieceFromIntaking())
                    .withName("requestIntakeUntilPiece");
        }

        /** Await a game piece from intaking. Ends immediately if scheduled while not intaking. */
        public Command awaitGamePieceFromIntaking() {
            return groundManager
                    .waitForState(GroundManagerStates.PREPARE_IDLE)
                    .onlyIf(() -> groundManager.getState() == GroundManagerStates.INTAKING);
        }

        /** Prepare for L1 score and await ready state. */
        public Command prepareL1AndAwaitReady() {
            return setState(GroundManagerStates.PREPARE_SCORE_L1)
                    .andThen(groundManager.waitForState(GroundManagerStates.READY_SCORE_L1))
                    .withName("requestL1PrepareAndAwaitReady");
        }

        /** Score L1 and await idle state. */
        public Command executeL1ScoreAndAwaitIdle() {
            return setState(GroundManagerStates.SCORE_L1)
                    .andThen(groundManager.waitForState(GroundManagerStates.PREPARE_IDLE))
                    .withName("requestL1ScoreAndAwaitIdle");
        }

        /** Execute handoff. */
        public Command executeHandoff() {
            return setState(GroundManagerStates.HANDOFF);
        }

        /** Execute inverted handoff. */
        public Command executeInvertedHandoff() {
            return setState(GroundManagerStates.INVERTED_HANDOFF);
        }

        /** Do nothing until interrupted externally. */
        public Command doNothing() {
            return Commands.idle(groundManager);
        }
    }

    private static GroundManager instance;

    public static GroundManager getInstance() {
        if (instance == null) instance = new GroundManager();
        return instance;
    }
}
