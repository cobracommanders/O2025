package frc.robot.subsystems.armManager;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.ArmManagerState.HandGamePieceState;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmState;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandState;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorState;

import java.util.function.Supplier;

public class ArmManager extends StateMachine<ArmManagerState> {
    private final Hand hand;
    private final ArmScheduler armScheduler;

    private final double ALGAE_DROP_TIME = 0.1;
    private final Debouncer algaeDroppedOrMissingDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

    public ArmManager(
            Hand hand,
            Elevator elevator,
            Arm arm) {
        super(ArmManagerState.START_POSITION, "ArmManager");
        this.hand = hand;

        this.armScheduler = new ArmScheduler(arm, elevator, hand);
    }

    public boolean isReadyToReturnToIdleAfterScoringCoral() {
        //TODO Does it do something else in auto?
        if (DriverStation.isAutonomous()) return false;
        //TODO Is this just measuring distance from the target scoring pose? If so, would being 0.25m to the left/right also allow the arm to move?
        return timeout(0.5) && AutoAlign.getInstance().getUsedScoringPose().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose().getTranslation()) >= 0.25;
    }

    public boolean isReadyToReturnToIdleAfterIntakingAlgae() {
        return AutoAlign.getInstance().getAlgaeDistance().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose().getTranslation()) >= 1.25;
    }

    public boolean atPosition() {
        return armScheduler.isReady();
    }

    public boolean isIdleState() {
        return getState().isIdleState();
    }

    public void overrideArmAcceleration(double armAcceleration) {
        armScheduler.overrideArmAcceleration(armAcceleration);
    }

    public void clearOverrideArmAcceleration() {
        armScheduler.clearOverrideArmAcceleration();
    }

    @Override
    protected void collectInputs() {
        DogLog.log("ArmManager/atPosition", atPosition());
    }

    protected ArmManagerState getNextState(ArmManagerState currentState) {
        ArmManagerState nextState = currentState;

        switch (currentState) {
            /* ******** START POSITION ******** */
            case START_POSITION -> {
                if (DriverStation.isEnabled()) nextState = ArmManagerState.PREPARE_IDLE_CORAL_UP;
            }

            /* ******** IDLE STATES ******** */
            case PREPARE_IDLE_EMPTY, PREPARE_IDLE_CORAL_UP, PREPARE_IDLE_CORAL_DOWN, PREPARE_IDLE_ALGAE -> {
                if (atPosition()) nextState = currentState.getPrepareToIdleState();
            }

            case IDLE_ALGAE -> {
                if (algaeDroppedOrMissingDebouncer.calculate(!hand.droppedAlgae()) && !Robot.isSimulation()) {
                    nextState = ArmManagerState.IDLE_ALGAE_DROPPED;
                }
            }

            case IDLE_ALGAE_DROPPED -> {
                if (timeout(ALGAE_DROP_TIME)) nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
            }

            case IDLE_EMPTY, IDLE_CORAL_UP, IDLE_CORAL_DOWN -> {/* Await Control */}

            case PREPARE_HANDOFF_LEFT, PREPARE_HANDOFF_MIDDLE, PREPARE_HANDOFF_RIGHT -> {
                if (atPosition()) {
                    nextState = currentState.getHandoffPrepareToCompleteState();
                }
            }

            case HANDOFF_LEFT, HANDOFF_MIDDLE, HANDOFF_RIGHT -> { /* Await Control */ }

            /* ******** INVERTED HANDOFF STATES ******** */
            case PREPARE_INVERTED_HANDOFF -> {
                if (atPosition()) nextState = ArmManagerState.READY_INVERTED_HANDOFF;
            }

            case READY_INVERTED_HANDOFF -> { /* Await Control */ }

            case EXECUTE_INVERTED_HANDOFF -> { /* Await Control */ }

            /* ******** CORAL SCORE STATES ******** */
            case PREPARE_L4_LEFT,
                 PREPARE_L3_LEFT,
                 PREPARE_L2_LEFT,
                 PREPARE_L4_RIGHT,
                 PREPARE_L3_RIGHT,
                 PREPARE_L2_RIGHT -> {
                if (atPosition()) {
                    nextState = currentState.getCoralPrepareToReadyState();
                }
            }

            case READY_L4_LEFT,
                 READY_L3_LEFT,
                 READY_L2_LEFT,
                 READY_L4_RIGHT,
                 READY_L3_RIGHT,
                 READY_L2_RIGHT -> {
                /* Await Control */
            }

            case SCORE_L4_LEFT,
                 SCORE_L3_LEFT,
                 SCORE_L2_LEFT,
                 SCORE_L4_RIGHT,
                 SCORE_L3_RIGHT,
                 SCORE_L2_RIGHT -> {
                if (atPosition()) {
                    nextState = currentState.getCoralScoreToFinishedState();
                }
            }

            case FINISHED_SCORE_L4_LEFT,
                 FINISHED_SCORE_L3_LEFT,
                 FINISHED_SCORE_L2_LEFT,
                 FINISHED_SCORE_L4_RIGHT,
                 FINISHED_SCORE_L3_RIGHT,
                 FINISHED_SCORE_L2_RIGHT -> {
                if (isReadyToReturnToIdleAfterScoringCoral()) {
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                }
            }


            /* ******** ALGAE INTAKE STATES ******** */
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT,
                 PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT,
                 PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT,
                 PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT,
                 PREPARE_INTAKE_GROUND_ALGAE -> {
                if (atPosition()) {
                    nextState = currentState.getAlgaeIntakePrepareToActiveState();
                }
            }

            case ACTIVE_INTAKE_HIGH_REEF_ALGAE_LEFT,
                 ACTIVE_INTAKE_HIGH_REEF_ALGAE_RIGHT,
                 ACTIVE_INTAKE_LOW_REEF_ALGAE_LEFT,
                 ACTIVE_INTAKE_LOW_REEF_ALGAE_RIGHT -> {
                if (isReadyToReturnToIdleAfterIntakingAlgae()) {
                    nextState = ArmManagerState.IDLE_ALGAE;
                }
            }

            case ACTIVE_INTAKE_GROUND_ALGAE -> {
                if (hand.hasAlgaeForIntake()) {
                    nextState = ArmManagerState.PREPARE_IDLE_ALGAE;
                }
            }


            /* ******** ALGAE SCORE STATES ******** */
            case PREPARE_SCORE_ALGAE_NET_LEFT,
                 PREPARE_SCORE_ALGAE_NET_RIGHT -> {
                if (atPosition()) {
                    nextState = currentState.getAlgaeNetPrepareToReadyState();
                }
            }

            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (atPosition()) {
                    nextState = ArmManagerState.READY_SCORE_ALGAE_PROCESSOR;
                }
            }

            case READY_SCORE_ALGAE_NET_LEFT,
                 READY_SCORE_ALGAE_NET_RIGHT,
                 READY_SCORE_ALGAE_PROCESSOR -> { /* Await Control */ }

            case SCORE_ALGAE_NET_LEFT,
                 SCORE_ALGAE_NET_RIGHT,
                 SCORE_ALGAE_PROCESSOR -> {
                if (timeout(1)) {
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                }
            }

            /* ******** LOLLIPOP INTAKE STATES ******** */
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (atPosition()) {
                    nextState = ArmManagerState.ACTIVE_INTAKE_LOLLIPOP;
                }
            }

            case ACTIVE_INTAKE_LOLLIPOP -> { /* Await Control */ }


            /* ******** CLIMB STATES ******** */
            case PREPARE_CLIMB -> {
                if (atPosition()) {
                    nextState = ArmManagerState.READY_CLIMB;
                }
            }

            case READY_CLIMB -> { /* Await Control */ }
        }

        return nextState;
    }

    private void requestState(ArmState armState, ElevatorState elevatorState, HandState handState) {
        this.requestState(armState, elevatorState, handState, Constants.ArmConstants.DefaultMotionMagicAcceleration);
    }
    private void requestState(ArmState armState, ElevatorState elevatorState, HandState handState, double armAcceleration) {
        armScheduler.scheduleStates(armState, elevatorState, handState, armAcceleration);
    }

    public boolean isArmUp() {
        return armScheduler.isArmUp();
    }

    @Override
    protected void afterTransition(ArmManagerState newState) {
        switch (newState) {
            /* ******** START POSITION ******** */
            case START_POSITION -> { /* N/A */ }

            /* ******** IDLE STATES ******** */
            case PREPARE_IDLE_EMPTY, IDLE_EMPTY ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.IDLE_EMPTY, HandState.IDLE_EMPTY);

            case PREPARE_IDLE_CORAL_UP, IDLE_CORAL_UP ->
                    requestState(ArmState.IDLE_CORAL_UP, ElevatorState.IDLE_CORAL_UP, HandState.IDLE_CORAL);

            case PREPARE_IDLE_CORAL_DOWN, IDLE_CORAL_DOWN ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.IDLE_EMPTY, HandState.IDLE_CORAL);

            case PREPARE_IDLE_ALGAE, IDLE_ALGAE ->
                    requestState(ArmState.IDLE_ALGAE, ElevatorState.IDLE, HandState.IDLE_ALGAE);
            case IDLE_ALGAE_DROPPED -> requestState(ArmState.IDLE_ALGAE, ElevatorState.IDLE, HandState.CLEAR_ALGAE);

            /* ******** HANDOFF STATES ******** */
            case PREPARE_HANDOFF_LEFT, PREPARE_HANDOFF_MIDDLE, PREPARE_HANDOFF_RIGHT,
                 HANDOFF_LEFT, HANDOFF_MIDDLE, HANDOFF_RIGHT ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.HANDOFF);


            /* ******** INVERTED HANDOFF STATES ******** */
            case PREPARE_INVERTED_HANDOFF, READY_INVERTED_HANDOFF ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.IDLE_CORAL);
            case EXECUTE_INVERTED_HANDOFF ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.INVERTED_HANDOFF);


            /* ******** CORAL SCORE STATES ******** */
            case PREPARE_L4_LEFT, READY_L4_LEFT ->
                    requestState(ArmState.PREPARE_L4_LEFT, ElevatorState.PREPARE_L4, HandState.IDLE_CORAL);
            case PREPARE_L3_LEFT, READY_L3_LEFT ->
                    requestState(ArmState.PREPARE_L3_LEFT, ElevatorState.PREPARE_L3, HandState.IDLE_CORAL);
            case PREPARE_L2_LEFT, READY_L2_LEFT ->
                    requestState(ArmState.PREPARE_L2_LEFT, ElevatorState.PREPARE_L2, HandState.IDLE_CORAL);
            case PREPARE_L4_RIGHT, READY_L4_RIGHT ->
                    requestState(ArmState.PREPARE_L4_RIGHT, ElevatorState.PREPARE_L4, HandState.IDLE_CORAL);
            case PREPARE_L3_RIGHT, READY_L3_RIGHT ->
                    requestState(ArmState.PREPARE_L3_RIGHT, ElevatorState.PREPARE_L3, HandState.IDLE_CORAL);
            case PREPARE_L2_RIGHT, READY_L2_RIGHT ->
                    requestState(ArmState.PREPARE_L2_RIGHT, ElevatorState.PREPARE_L2, HandState.IDLE_CORAL);

            case SCORE_L4_LEFT -> requestState(ArmState.SCORE_L4_LEFT, ElevatorState.SCORE_L4, HandState.IDLE_CORAL, 6.0);
            case SCORE_L3_LEFT -> requestState(ArmState.SCORE_L3_LEFT, ElevatorState.SCORE_L3, HandState.IDLE_CORAL, 6.0);
            case SCORE_L2_LEFT -> requestState(ArmState.SCORE_L2_LEFT, ElevatorState.SCORE_L2, HandState.IDLE_CORAL, 6.0);
            case SCORE_L4_RIGHT -> requestState(ArmState.SCORE_L4_RIGHT, ElevatorState.SCORE_L4, HandState.IDLE_CORAL, 6.0);
            case SCORE_L3_RIGHT -> requestState(ArmState.SCORE_L3_RIGHT, ElevatorState.SCORE_L3, HandState.IDLE_CORAL, 6.0);
            case SCORE_L2_RIGHT -> requestState(ArmState.SCORE_L2_RIGHT, ElevatorState.SCORE_L2, HandState.IDLE_CORAL, 6.0);

            case FINISHED_SCORE_L4_LEFT ->
                    requestState(ArmState.SCORE_L4_LEFT, ElevatorState.SCORE_L4, HandState.SCORE_CORAL);
            case FINISHED_SCORE_L3_LEFT ->
                    requestState(ArmState.SCORE_L3_LEFT, ElevatorState.SCORE_L3, HandState.SCORE_CORAL);
            case FINISHED_SCORE_L2_LEFT ->
                    requestState(ArmState.SCORE_L2_LEFT, ElevatorState.SCORE_L2, HandState.SCORE_CORAL);
            case FINISHED_SCORE_L4_RIGHT ->
                    requestState(ArmState.SCORE_L4_RIGHT, ElevatorState.SCORE_L4, HandState.SCORE_CORAL);
            case FINISHED_SCORE_L3_RIGHT ->
                    requestState(ArmState.SCORE_L3_RIGHT, ElevatorState.SCORE_L3, HandState.SCORE_CORAL);
            case FINISHED_SCORE_L2_RIGHT ->
                    requestState(ArmState.SCORE_L2_RIGHT, ElevatorState.SCORE_L2, HandState.SCORE_CORAL);


            /* ******** ALGAE INTAKE STATES ******** */
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT ->
                    requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_LEFT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT ->
                    requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_RIGHT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT ->
                    requestState(ArmState.INTAKE_LOW_REEF_ALGAE_LEFT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT ->
                    requestState(ArmState.INTAKE_LOW_REEF_ALGAE_RIGHT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_GROUND_ALGAE ->
                    requestState(ArmState.INTAKE_GROUND_ALGAE, ElevatorState.GROUND_ALGAE, HandState.INTAKE_GROUND_ALGAE);


            /* ******** ALGAE SCORE STATES ******** */
            case PREPARE_SCORE_ALGAE_NET_LEFT, READY_SCORE_ALGAE_NET_LEFT ->
                    requestState(ArmState.ALGAE_NET_LEFT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_NET_RIGHT, READY_SCORE_ALGAE_NET_RIGHT ->
                    requestState(ArmState.ALGAE_NET_RIGHT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_PROCESSOR, READY_SCORE_ALGAE_PROCESSOR ->
                    requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.IDLE_ALGAE);
            case SCORE_ALGAE_NET_LEFT -> hand.setState(HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_NET_RIGHT -> hand.setState(HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_PROCESSOR ->
                    requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.SCORE_ALGAE_PROCESSOR);

            /* ******** LOLLIPOP INTAKE STATES ******** */
            case PREPARE_INTAKE_LOLLIPOP -> requestState(ArmState.LOLLIPOP, ElevatorState.LOLLIPOP, HandState.LOLLIPOP);
            case ACTIVE_INTAKE_LOLLIPOP -> requestState(ArmState.LOLLIPOP, ElevatorState.LOLLIPOP, HandState.LOLLIPOP);

            /* ******** CLIMB STATES ******** */
            case PREPARE_CLIMB, READY_CLIMB -> requestState(ArmState.CLIMB, ElevatorState.IDLE, HandState.CLEAR_ALGAE);
        }
    }

    public HandGamePieceState getCurrentGamePiece() {
        return getState().handGamePieceState;
    }

    public void requestHandoff(CoralDetectorState coralPosition) {
        if (getState().handGamePieceState.isNone()) {
            setStateFromRequest(ArmManagerState.getHandoffPrepareFromCoralPosition(coralPosition));
        }
    }

    public boolean isHandoffReady() {
        return getState().isHandoffReadyState();
    }

    public void requestInvertedHandoff() {
        if (getState().handGamePieceState.isCoral()) {
            setStateFromRequest(ArmManagerState.PREPARE_INVERTED_HANDOFF);
        }
    }

    public boolean isReadyToExecuteInvertedHandoff() {
        return getState() == ArmManagerState.READY_INVERTED_HANDOFF;
    }

    public void requestInvertedHandoffExecution() {
        if (isReadyToExecuteInvertedHandoff()) {
            setStateFromRequest(ArmManagerState.EXECUTE_INVERTED_HANDOFF);
        }
    }

    public void requestClimb() {
        setStateFromRequest(ArmManagerState.PREPARE_CLIMB);
    }

    public void requestIdle() {
        ArmManagerState state = switch (getState().handGamePieceState) {
            case CORAL -> armScheduler.isArmUp() ? ArmManagerState.IDLE_CORAL_UP : ArmManagerState.IDLE_CORAL_DOWN;
            case ALGAE -> ArmManagerState.IDLE_ALGAE;
            case NONE -> ArmManagerState.IDLE_EMPTY;
        };
        setStateFromRequest(state);
    }

    public void requestCoralIdle() {
        setStateFromRequest(ArmManagerState.IDLE_CORAL_DOWN);
    }

    public void requestIdleClearGamePiece() {
        setStateFromRequest(ArmManagerState.IDLE_ALGAE_DROPPED);
    }

    public void requestReefAlgaeIntake(RobotScoringSide side, boolean top) {
        setStateFromRequest(ArmManagerState.getAlgaeIntakePrepare(side, top));
    }

    public void requestGroundAlgaeIntake() {
        setStateFromRequest(ArmManagerState.PREPARE_INTAKE_GROUND_ALGAE);
    }

    public void requestLollipopIntake() {
        setStateFromRequest(ArmManagerState.PREPARE_INTAKE_LOLLIPOP);
    }

    public boolean isLollipopIntakeReady() {
        return getState() == ArmManagerState.ACTIVE_INTAKE_LOLLIPOP;
    }

    public void requestCoralPrepare(RobotScoringSide robotSide, FieldConstants.PipeScoringLevel scoringLevel) {
        ArmManagerState coralPrepareState = ArmManagerState.getCoralPrepareScore(robotSide, scoringLevel);
        if (getState().handGamePieceState.isCoral() && getState() != coralPrepareState.getCoralPrepareToReadyState()) {
            setStateFromRequest(coralPrepareState);
        }
    }

    public boolean isReadyToScoreCoral() {
        return getState().isCoralReadyToScoreState();
    }

    public void requestCoralScoreExecution() {
        setStateFromRequest(getState().getCoralReadyToScoreState());
    }

    public boolean isCoralScoreFinished() {
        return getState().isCoralScoreFinishedState();
    }

    public void requestAlgaeNetScore(RobotScoringSide robotSide) {
        if (getState().handGamePieceState.isAlgae()) {
            setStateFromRequest(ArmManagerState.getNetPrepareScore(robotSide));
        }
    }

    public boolean isReadyToScoreAlgaeNet() {
        return getState().isNetReadyState();
    }

    public void requestAlgaeNetScoreExecution() {
        if (isReadyToScoreAlgaeNet()) {
            setStateFromRequest(getState().getAlgaeNetReadyToScoreState());
        }
    }

    public void requestProcessorScore() {
        if (getState().handGamePieceState.isAlgae()) {
            setStateFromRequest(ArmManagerState.PREPARE_SCORE_ALGAE_PROCESSOR);
        }
    }

    public boolean isReadyToScoreProcessor() {
        return getState() == ArmManagerState.READY_SCORE_ALGAE_PROCESSOR;
    }

    public void requestProcessorScoreExecution() {
        if (isReadyToScoreProcessor()) {
            setStateFromRequest(ArmManagerState.SCORE_ALGAE_PROCESSOR);
        }
    }

    public static class CommandWrapper {
        private final ArmManager armManager;

        public CommandWrapper(ArmManager armManager) {
            this.armManager = armManager;
        }

        public boolean isArmReadyToScoreCoral() {
            return armManager.isReadyToScoreCoral();
        }

        public boolean isArmIdle() {
            return armManager.isIdleState();
        }


        /**
         * Request algae processor score and await ready state.
         */
        public Command requestAlgaeProcessorPrepareAndAwaitReady() {
            return armManager.runOnce(armManager::requestProcessorScore)
                    .andThen(Commands.waitUntil(armManager::isReadyToScoreProcessor))
                    .withName("requestAlgaeProcessorScoreAndAwaitReady");
        }

        /**
         * Execute algae processor score and await idle state.
         */
        public Command executeAlgaeProcessorScoreAndAwaitIdle() {
            return armManager.runOnce(armManager::requestProcessorScoreExecution)
                    .andThen(armManager.waitForState(ArmManagerState.PREPARE_IDLE_EMPTY))
                    .withName("executeAlgaeProcessorScoreAndAwaitIdle");
        }

        /**
         * Request algae net score and await ready state.
         * Will ONLY use the scoring side from the supplier at the time of scheduling.
         */
        public Command requestAlgaeNetPrepareAndAwaitReady(Supplier<RobotScoringSide> side) {
            return armManager.runOnce(() -> armManager.requestAlgaeNetScore(side.get()))
                    .andThen(Commands.waitUntil(armManager::isReadyToScoreAlgaeNet))
                    .withName("requestAlgaeNetScoreAndAwaitReady");
        }

        /**
         * Execute algae net score and await idle state.
         */
        public Command executeAlgaeNetScoreAndAwaitIdle() {
            return armManager.runOnce(armManager::requestAlgaeNetScoreExecution)
                    .andThen(armManager.waitForState(ArmManagerState.PREPARE_IDLE_EMPTY))
                    .withName("executeAlgaeNetScoreAndAwaitIdle");
        }

        /**
         * Request climb and do nothing.
         */
        public Command requestClimbAndDoNothing() {
            return armManager.runOnce(armManager::requestClimb)
                    .andThen(doNothing())
                    .withName("requestClimbAndDoNothing");
        }

        /**
         * Prepare for coral score and await ready state.
         * Will constantly update the scoring side and level based on the value of the supplier.
         */
        public Command requestCoralPrepareAndAwaitReady(Supplier<RobotScoringSide> side,
                                                        Supplier<FieldConstants.PipeScoringLevel> scoringLevel) {
            return armManager.runOnce(() -> armManager.requestCoralPrepare(side.get(), scoringLevel.get()))
                    .repeatedly().until(armManager::isReadyToScoreCoral)
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                    .withName("requestCoralPrepareAndAwaitReady");
        }

        /**
         * Execute coral score and await movement completion.
         */
        public Command executeCoralScoreAndAwaitComplete() {
            return armManager.runOnce(armManager::requestCoralScoreExecution)
                    .andThen(Commands.waitUntil(armManager::isCoralScoreFinished))
                    .onlyIf(armManager::isReadyToScoreCoral)
                    .withName("executeCoralScoreAndAwaitComplete");
        }

        /**
         * Move the arm to applicable idle state based on the current game piece state.
         */
        public Command idleAndAwaitReady() {
            return armManager
                    .runOnce(armManager::requestIdle)
                    .andThen(Commands.waitUntil(armManager::isIdleState))
                    .withName("idleAndAwaitReady");
        }

        /**
         * Request idle clear game piece.
         */
        public Command idleClearGamePieceAndAwaitReady() {
            return armManager.runOnce(armManager::requestIdleClearGamePiece)
                    .andThen(Commands.waitUntil(armManager::isIdleState))
                    .withName("requestIdleClearGamePieceAndAwaitReady");
        }

        /**
         * Request algae reef intake and await idle state after collecting a game piece.
         * Will constantly update the scoring side based on the value of the supplier.
         */
        public Command requestAlgaeReefIntakeAndAwaitIdle(Supplier<RobotScoringSide> side, boolean top) {
            return armManager.runOnce(() -> armManager.requestReefAlgaeIntake(side.get(), top))
                    .repeatedly()
                    .withDeadline(armManager.waitForState(ArmManagerState.PREPARE_IDLE_ALGAE))
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                    .withName("requestAlgaeReefIntakeAndAwaitIdle");
        }

        /**
         * Request ground algae intake and await game piece.
         */
        public Command requestGroundAlgaeIntakeAndAwaitGamePiece() {
            return armManager.runOnce(armManager::requestGroundAlgaeIntake)
                    .andThen(armManager.waitForState(ArmManagerState.ACTIVE_INTAKE_GROUND_ALGAE))
                    .andThen(armManager.waitForState(ArmManagerState.PREPARE_IDLE_ALGAE))
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                    .withName("requestGroundAlgaeIntakeAndAwaitGamePiece");
        }

        /**
         * Request lollipop intake and await ready state.
         */
        public Command requestLollipopIntakeAndAwaitReady() {
            return armManager
                    .runOnce(armManager::requestLollipopIntake)
                    .andThen(Commands.waitUntil(armManager::isLollipopIntakeReady))
                    .withName("requestLollipopIntakeAndAwaitReady");
        }

        /**
         * Prepare for handoff and await ready state. Updates to the supplied position until a ready state is reached.
         */
        public Command requestHandoffAndAwaitReady(Supplier<CoralDetectorState> position) {
            return armManager
                    // .run() continually calls the given block until interrupted
                    // this lets the position be constantly updated in case it changes for some reason
                    .runOnce(() -> armManager.requestHandoff(position.get()))
                    // Interrupt when ready
                    .andThen(Commands.waitUntil(armManager::isHandoffReady))
                    // Only let this run if the hand is not holding a game piece
                    .onlyIf(() -> armManager.getState().handGamePieceState.isNone())
                    .withName("requestHandoffAndAwaitReady");
        }

        public Command completeHandoffAndCoralIdle() {
            return Commands.runOnce(armManager::requestCoralIdle).andThen(Commands.waitUntil(armManager::isIdleState));
        }

        /**
         * Prepare for inverted handoff and await ready state.
         */
        public Command requestInvertedHandoffAndAwaitReady() {
            return armManager
                    .runOnce(armManager::requestInvertedHandoff)
                    .until(armManager::isHandoffReady)
                    // Only let this run if the hand is holding a coral
                    .onlyIf(() -> armManager.getState().handGamePieceState.isCoral())
                    .withName("requestInvertedHandoffAndAwaitReady");
        }

        public Command executeInvertedHandoff() {
            return armManager.runOnce(armManager::requestInvertedHandoffExecution)
                    .withName("executeInvertedHandoff");
        }

        /**
         * Do nothing until interrupted externally.
         */
        public Command doNothing() {
            return Commands.idle(armManager);
        }

        public HandGamePieceState getCurrentGamePiece() {
            return armManager.getCurrentGamePiece();
        }

        public Command overrideArmAcceleration(double armAcceleration) {
            return Commands.runOnce(() -> armManager.overrideArmAcceleration(armAcceleration));
        }

        public Command clearOverrideArmAcceleration() {
            return Commands.runOnce(armManager::clearOverrideArmAcceleration);
        }

        public boolean currentGamePieceIsNone() {
            return getCurrentGamePiece().isNone();
        }

        public boolean currentGamePieceIsCoral() {
            return getCurrentGamePiece().isCoral();
        }

        public boolean currentGamePieceIsAlgae() {
            return getCurrentGamePiece().isAlgae();
        }
    }
}
