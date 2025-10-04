package frc.robot.subsystems.armManager;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.ArmManagerState.HandGamePieceState;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmState;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandState;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorState;

public class ArmManager extends StateMachine<ArmManagerState> {
    private final Hand hand;
    private final Elevator elevator;
    private final Arm arm;
    private final CoralDetector coralDetector = CoralDetector.getInstance();
    private final ArmScheduler armScheduler;

    private final double ALGAE_DROP_TIME = 0.1;
    private final Debouncer algaeDroppedOrMissingDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

    public ArmManager(
            Hand hand,
            Elevator elevator,
            Arm arm
    ) {
        super(ArmManagerState.START_POSITION);
        this.hand = hand;
        this.elevator = elevator;
        this.arm = arm;

        this.armScheduler = new ArmScheduler(arm, elevator, hand);
    }

    public boolean isReadyToReturnToIdleAfterScoringCoral() {
        //TODO Does it do something else in auto?
        if (DriverStation.isAutonomous()) return false;
        //TODO Is this just measuring distance from the target scoring pose? If so, would being 0.25m to the left/right also allow the arm to move?
        return timeout(0.5) && AutoAlign.getInstance().usedScoringPose.getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose().getTranslation()) >= 0.25;
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

    @Override
    protected void collectInputs() {
        DogLog.log("ArmManager/atPosition", atPosition());
    }

    protected ArmManagerState getNextState(ArmManagerState currentState) {
        ArmManagerState nextState = currentState;

        switch (currentState) {
            /* ******** START POSITION ******** */
            case START_POSITION -> {
                if (DriverStation.isEnabled()) nextState = ArmManagerState.PREPARE_IDLE_CORAL;
            }

            /* ******** IDLE STATES ******** */
            case PREPARE_IDLE_EMPTY, PREPARE_IDLE_CORAL, PREPARE_IDLE_ALGAE -> {
                if (atPosition()) nextState = currentState.getPrepareToIdleState();
            }

            case IDLE_ALGAE -> {
                if (algaeDroppedOrMissingDebouncer.calculate(!hand.hasAlgae()) && !Robot.isSimulation()) {
                    nextState = ArmManagerState.IDLE_ALGAE_DROPPED;
                }
            }

            case IDLE_ALGAE_DROPPED -> {
                if (timeout(ALGAE_DROP_TIME)) nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
            }

            case IDLE_EMPTY -> {
                if (coralDetector.hasCoral()) {
                    nextState = ArmManagerState.getHandoffPreemptiveFromCoralPosition(coralDetector.getState());
                }
            }

            case IDLE_CORAL -> {/* Await Control */}


            /* ******** HANDOFF STATES ******** */
            case PREEMPTIVE_HANDOFF_LEFT, PREEMPTIVE_HANDOFF_MIDDLE, PREEMPTIVE_HANDOFF_RIGHT -> {
                CoralDetectorState coralPosition = coralDetector.getState();
                boolean hasCoral = coralPosition != CoralDetectorState.NONE;
                ArmManagerState targetPreemptiveState = ArmManagerState.getHandoffPreemptiveFromCoralPosition(coralPosition);

                if (!hasCoral) {
                    // If no coral, go back to idle empty
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                } else if (targetPreemptiveState != currentState) {
                    // If the coral position has changed, prepare to move to the new position
                    nextState = targetPreemptiveState;
                } else {
                    // The coral is present and the arm is in the correct state, do nothing
                }
            }

            case PREPARE_HANDOFF_LEFT, PREPARE_HANDOFF_MIDDLE, PREPARE_HANDOFF_RIGHT -> {
                if (atPosition()) {
                    nextState = currentState.getHandoffPrepareToReadyState();
                }
            }

            case READY_HANDOFF_LEFT, READY_HANDOFF_MIDDLE, READY_HANDOFF_RIGHT -> {
                boolean hasCoral = coralDetector.hasCoral();

                if (!hasCoral) {
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                }
            }

            case EXECUTE_HANDOFF_LEFT, EXECUTE_HANDOFF_MIDDLE, EXECUTE_HANDOFF_RIGHT -> {
                if (timeout(RequestManager.HANDOFF_TIME)) {
                    nextState = ArmManagerState.PREPARE_IDLE_CORAL;
                }
            }


            /* ******** INVERTED HANDOFF STATES ******** */
            case PREPARE_INVERTED_HANDOFF -> {
                if (atPosition()) nextState = ArmManagerState.READY_INVERTED_HANDOFF;
            }

            case READY_INVERTED_HANDOFF -> { /* Await Control */ }

            case EXECUTE_INVERTED_HANDOFF -> {
                if (timeout(RequestManager.INVERTED_HANDOFF_TIME)) {
                    nextState = ArmManagerState.IDLE_EMPTY;
                }
            }


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
                if (hand.hasAlgae()) {
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
        armScheduler.scheduleStates(armState, elevatorState, handState);
    }

    @Override
    protected void afterTransition(ArmManagerState newState) {
        switch (newState) {
            /* ******** START POSITION ******** */
            case START_POSITION -> { /* N/A */ }

            /* ******** IDLE STATES ******** */
            case PREPARE_IDLE_EMPTY, IDLE_EMPTY ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.PREPARE_HANDOFF, HandState.IDLE_EMPTY);
            case PREPARE_IDLE_CORAL, IDLE_CORAL ->
                    requestState(ArmState.IDLE_CORAL, ElevatorState.IDLE_CORAL, HandState.IDLE_CORAL);

            case PREPARE_IDLE_ALGAE, IDLE_ALGAE ->
                    requestState(ArmState.IDLE_ALGAE, ElevatorState.IDLE, HandState.IDLE_ALGAE);
            case IDLE_ALGAE_DROPPED -> requestState(ArmState.IDLE_ALGAE, ElevatorState.IDLE, HandState.CLEAR_ALGAE);


            /* ******** HANDOFF STATES ******** */
            case PREPARE_HANDOFF_LEFT, PREEMPTIVE_HANDOFF_LEFT ->
                    requestState(ArmState.HANDOFF_LEFT, ElevatorState.PREPARE_HANDOFF, HandState.IDLE_EMPTY);
            case PREPARE_HANDOFF_MIDDLE, PREEMPTIVE_HANDOFF_MIDDLE ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.PREPARE_HANDOFF, HandState.IDLE_EMPTY);
            case PREPARE_HANDOFF_RIGHT, PREEMPTIVE_HANDOFF_RIGHT ->
                    requestState(ArmState.HANDOFF_RIGHT, ElevatorState.PREPARE_HANDOFF, HandState.IDLE_EMPTY);

            case READY_HANDOFF_LEFT ->
                    requestState(ArmState.HANDOFF_LEFT, ElevatorState.PREPARE_HANDOFF, HandState.HANDOFF);
            case READY_HANDOFF_MIDDLE ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.PREPARE_HANDOFF, HandState.HANDOFF);
            case READY_HANDOFF_RIGHT ->
                    requestState(ArmState.HANDOFF_RIGHT, ElevatorState.PREPARE_HANDOFF, HandState.HANDOFF);

            case EXECUTE_HANDOFF_LEFT -> requestState(ArmState.HANDOFF_LEFT, ElevatorState.HANDOFF, HandState.HANDOFF);
            case EXECUTE_HANDOFF_MIDDLE ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.HANDOFF);
            case EXECUTE_HANDOFF_RIGHT ->
                    requestState(ArmState.HANDOFF_RIGHT, ElevatorState.HANDOFF, HandState.HANDOFF);


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

            case SCORE_L4_LEFT -> requestState(ArmState.SCORE_L4_LEFT, ElevatorState.SCORE_L4, HandState.SCORE_CORAL);
            case SCORE_L3_LEFT -> requestState(ArmState.SCORE_L3_LEFT, ElevatorState.SCORE_L3, HandState.SCORE_CORAL);
            case SCORE_L2_LEFT -> requestState(ArmState.SCORE_L2_LEFT, ElevatorState.SCORE_L2, HandState.SCORE_CORAL);
            case SCORE_L4_RIGHT -> requestState(ArmState.SCORE_L4_RIGHT, ElevatorState.SCORE_L4, HandState.SCORE_CORAL);
            case SCORE_L3_RIGHT -> requestState(ArmState.SCORE_L3_RIGHT, ElevatorState.SCORE_L3, HandState.SCORE_CORAL);
            case SCORE_L2_RIGHT -> requestState(ArmState.SCORE_L2_RIGHT, ElevatorState.SCORE_L2, HandState.SCORE_CORAL);


            /* ******** ALGAE INTAKE STATES ******** */
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT, ACTIVE_INTAKE_HIGH_REEF_ALGAE_LEFT ->
                    requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_LEFT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT, ACTIVE_INTAKE_HIGH_REEF_ALGAE_RIGHT ->
                    requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_RIGHT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT, ACTIVE_INTAKE_LOW_REEF_ALGAE_LEFT ->
                    requestState(ArmState.INTAKE_LOW_REEF_ALGAE_LEFT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT, ACTIVE_INTAKE_LOW_REEF_ALGAE_RIGHT ->
                    requestState(ArmState.INTAKE_LOW_REEF_ALGAE_RIGHT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_GROUND_ALGAE, ACTIVE_INTAKE_GROUND_ALGAE ->
                    requestState(ArmState.INTAKE_GROUND_ALGAE, ElevatorState.GROUND_ALGAE, HandState.INTAKE_GROUND_ALGAE);



            /* ******** ALGAE SCORE STATES ******** */
            case PREPARE_SCORE_ALGAE_NET_LEFT, READY_SCORE_ALGAE_NET_LEFT ->
                    requestState(ArmState.ALGAE_NET_LEFT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_NET_RIGHT, READY_SCORE_ALGAE_NET_RIGHT ->
                    requestState(ArmState.ALGAE_NET_RIGHT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_PROCESSOR, READY_SCORE_ALGAE_PROCESSOR ->
                    requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.IDLE_ALGAE);
            case SCORE_ALGAE_NET_LEFT ->
                    requestState(ArmState.ALGAE_NET_LEFT, ElevatorState.ALGAE_NET, HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_NET_RIGHT ->
                    requestState(ArmState.ALGAE_NET_RIGHT, ElevatorState.ALGAE_NET, HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_PROCESSOR ->
                    requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.SCORE_ALGAE_PROCESSOR);


            /* ******** LOLLIPOP INTAKE STATES ******** */
            case PREPARE_INTAKE_LOLLIPOP, ACTIVE_INTAKE_LOLLIPOP ->
                    requestState(ArmState.LOLLIPOP, ElevatorState.LOLLIPOP, HandState.LOLLIPOP);


            /* ******** CLIMB STATES ******** */
            case PREPARE_CLIMB, READY_CLIMB -> requestState(ArmState.CLIMB, ElevatorState.IDLE, HandState.CLEAR_ALGAE);
        }
    }

    public HandGamePieceState getSimHandGamePiece() {
        return getState().handGamePieceState;
    }

    public void requestHandoff() {
        if (getState().handGamePieceState == HandGamePieceState.NONE) {
            setStateFromRequest(ArmManagerState.getHandoffPrepareFromCoralPosition(coralDetector.getState()));
        }
    }

    public boolean isReadyToExecuteHandoff() {
        return getState().isHandoffReadyState();
    }

    public void requestHandoffExecution() {
        if (isReadyToExecuteHandoff()) {
            setStateFromRequest(getState().getHandoffReadyToExecuteState());
        }
    }

    public void requestInvertedHandoff() {
        if (getState().handGamePieceState == HandGamePieceState.CORAL) {
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
        setStateFromRequest(ArmManagerState.getIdleStateFor(getState().handGamePieceState));
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

    public void requestCoralPrepare(RobotScoringSide robotSide, FieldConstants.PipeScoringLevel scoringLevel) {
        if (getState().handGamePieceState == HandGamePieceState.CORAL) {
            setStateFromRequest(ArmManagerState.getCoralPrepareScore(robotSide, scoringLevel));
        }
    }

    public boolean isReadyToScoreCoral() {
        return getState().isCoralReadyToScoreState();
    }

    public void requestCoralScoreExecution() {
        setStateFromRequest(getState().getCoralReadyToScoreState());
    }

    public void requestAlgaeNetScore(RobotScoringSide robotSide) {
        if (getState().handGamePieceState == HandGamePieceState.ALGAE) {
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
        if (getState().handGamePieceState == HandGamePieceState.ALGAE) {
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
}
