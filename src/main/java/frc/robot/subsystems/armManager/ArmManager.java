package frc.robot.subsystems.armManager;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.StateMachine;
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

    private final double HANDOFF_TIME = 0.2;
    private final double INVERTED_HANDOFF_TIME = 0.2;
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

        this.armScheduler = new ArmScheduler(hand, elevator, arm);
    }

    public boolean isReadyToReturnToIdleAfterScoringCoral() {
        //TODO Does it do something else in auto?
        if (DriverStation.isAutonomous()) return false;
        //TODO Is this just measuring distance from the target scoring pose? If so, would being 0.25m to the left/right also allow the arm to move?
        return AutoAlign.getInstance().usedScoringPose.getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 0.25;
    }

    public boolean isReadyToReturnToIdleAfterIntakingAlgae() {
        return AutoAlign.getInstance().getAlgaeDistance().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 1.25;
    }

    public boolean atPosition() {
        return armScheduler.atPosition();
    }

    public boolean isReadyToExecuteHandoff() {
        return getState().isHandoffReadyState();
    }

    public void requestHandoffExecution() {
        if (isReadyToExecuteHandoff()) {
            setStateFromRequest(getState().getHandoffReadyToExecuteState());
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
                if (algaeDroppedOrMissingDebouncer.calculate(!hand.hasAlgae())) {
                    nextState = ArmManagerState.IDLE_ALGAE_DROPPED;
                }
            }

            case IDLE_ALGAE_DROPPED -> {
                if (timeout(ALGAE_DROP_TIME)) nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
            }

            case IDLE_EMPTY -> {
                if (coralDetector.hasCoral()) {
                    nextState = ArmManagerState.getHandoffPrepareFromCoralPosition(coralDetector.getState());
                }
            }

            case IDLE_CORAL -> {/* Await Control */}


            /* ******** HANDOFF STATES ******** */
            case PREPARE_HANDOFF_LEFT, PREPARE_HANDOFF_MIDDLE, PREPARE_HANDOFF_RIGHT -> {
                CoralDetectorState coralPosition = coralDetector.getState();
                boolean hasCoral = coralPosition != CoralDetectorState.NONE;
                ArmManagerState targetPrepareState = ArmManagerState.getHandoffPrepareFromCoralPosition(coralPosition);

                if (!hasCoral) {
                    // If no coral, go back to idle empty
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                } else if (targetPrepareState != currentState) {
                    // If the coral position has changed, prepare to move to the new position
                    nextState = targetPrepareState;
                } else if (atPosition()) {
                    // If the arm is at position, wait for the intake to be in the right place
                    nextState = currentState.getHandoffPrepareToReadyState();
                } else {
                    // The coral is present and the arm is moving to the right place, do nothing
                }
            }

            case READY_HANDOFF_LEFT, READY_HANDOFF_MIDDLE, READY_HANDOFF_RIGHT -> {
                CoralDetectorState coralPosition = coralDetector.getState();
                boolean hasCoral = coralPosition != CoralDetectorState.NONE;
                ArmManagerState targetReadyState = ArmManagerState.getHandoffReadyFromCoralPosition(coralPosition);

                if (!hasCoral) {
                    // If no coral, go back to idle empty
                    nextState = ArmManagerState.PREPARE_IDLE_EMPTY;
                } else if (targetReadyState != currentState) {
                    // If the coral position has changed, prepare to move to the new position
                    nextState = targetReadyState;
                } else {
                    // The coral is present and the arm is in the right place, do nothing
                }
            }

            case EXECUTE_HANDOFF_LEFT, EXECUTE_HANDOFF_MIDDLE, EXECUTE_HANDOFF_RIGHT -> {
                if (timeout(HANDOFF_TIME)) {
                    nextState = ArmManagerState.PREPARE_IDLE_CORAL;
                }
            }


            /* ******** INVERTED HANDOFF STATES ******** */
            case PREPARE_INVERTED_HANDOFF -> {
                if (atPosition()) nextState = ArmManagerState.READY_INVERTED_HANDOFF;
            }

            case READY_INVERTED_HANDOFF -> { /* Await Control */ }

            case EXECUTE_INVERTED_HANDOFF -> {
                if (timeout(INVERTED_HANDOFF_TIME)) {
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
                 PREPARE_SCORE_ALGAE_NET_RIGHT,
                 PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (atPosition()) {
                    nextState = currentState.getAlgaeScorePrepareToReadyState();
                }
            }

            case READY_SCORE_ALGAE_NET_LEFT,
                 READY_SCORE_ALGAE_NET_RIGHT,
                 READY_SCORE_ALGAE_PROCESSOR -> {
                /* Await Control */
            }

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
        armScheduler.scheduleStates(armState, handState, elevatorState);
    }

    @Override
    protected void afterTransition(ArmManagerState newState) {
        switch (newState) {
            /* ******** START POSITION ******** */
            case START_POSITION -> { /* N/A */ }

            /* ******** IDLE STATES ******** */
            case PREPARE_IDLE_EMPTY, IDLE_EMPTY ->
                    requestState(ArmState.IDLE, ElevatorState.IDLE, HandState.IDLE_EMPTY);
            case PREPARE_IDLE_CORAL, IDLE_CORAL ->
                    requestState(ArmState.IDLE, ElevatorState.IDLE, HandState.IDLE_CORAL);

            case PREPARE_IDLE_ALGAE, IDLE_ALGAE ->
                    requestState(ArmState.IDLE, ElevatorState.IDLE, HandState.IDLE_ALGAE);
            case IDLE_ALGAE_DROPPED -> requestState(ArmState.IDLE, ElevatorState.IDLE, HandState.CLEAR_ALGAE);


            /* ******** HANDOFF STATES ******** */
            case PREPARE_HANDOFF_LEFT ->
                    requestState(ArmState.HANDOFF_LEFT, ElevatorState.HANDOFF, HandState.IDLE_EMPTY);
            case PREPARE_HANDOFF_MIDDLE ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.IDLE_EMPTY);
            case PREPARE_HANDOFF_RIGHT ->
                    requestState(ArmState.HANDOFF_RIGHT, ElevatorState.HANDOFF, HandState.IDLE_EMPTY);

            case READY_HANDOFF_LEFT, EXECUTE_HANDOFF_LEFT ->
                    requestState(ArmState.HANDOFF_LEFT, ElevatorState.HANDOFF, HandState.HANDOFF);
            case READY_HANDOFF_MIDDLE, EXECUTE_HANDOFF_MIDDLE ->
                    requestState(ArmState.HANDOFF_MIDDLE, ElevatorState.HANDOFF, HandState.HANDOFF);
            case READY_HANDOFF_RIGHT, EXECUTE_HANDOFF_RIGHT ->
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
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT, ACTIVE_INTAKE_HIGH_REEF_ALGAE_LEFT -> requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_LEFT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT, ACTIVE_INTAKE_HIGH_REEF_ALGAE_RIGHT -> requestState(ArmState.INTAKE_HIGH_REEF_ALGAE_RIGHT, ElevatorState.HIGH_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT, ACTIVE_INTAKE_LOW_REEF_ALGAE_LEFT -> requestState(ArmState.INTAKE_LOW_REEF_ALGAE_LEFT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT, ACTIVE_INTAKE_LOW_REEF_ALGAE_RIGHT -> requestState(ArmState.INTAKE_LOW_REEF_ALGAE_RIGHT, ElevatorState.LOW_REEF_ALGAE, HandState.INTAKE_REEF_ALGAE);
            case PREPARE_INTAKE_GROUND_ALGAE, ACTIVE_INTAKE_GROUND_ALGAE -> requestState(ArmState.INTAKE_GROUND_ALGAE, ElevatorState.GROUND_ALGAE, HandState.INTAKE_GROUND_ALGAE);



            /* ******** ALGAE SCORE STATES ******** */
            case PREPARE_SCORE_ALGAE_NET_LEFT, READY_SCORE_ALGAE_NET_LEFT -> requestState(ArmState.ALGAE_NET_LEFT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_NET_RIGHT, READY_SCORE_ALGAE_NET_RIGHT -> requestState(ArmState.ALGAE_NET_RIGHT, ElevatorState.ALGAE_NET, HandState.IDLE_ALGAE);
            case PREPARE_SCORE_ALGAE_PROCESSOR, READY_SCORE_ALGAE_PROCESSOR -> requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.IDLE_ALGAE);
            case SCORE_ALGAE_NET_LEFT -> requestState(ArmState.ALGAE_NET_LEFT, ElevatorState.ALGAE_NET, HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_NET_RIGHT -> requestState(ArmState.ALGAE_NET_RIGHT, ElevatorState.ALGAE_NET, HandState.SCORE_ALGAE_NET);
            case SCORE_ALGAE_PROCESSOR -> requestState(ArmState.ALGAE_PROCESSOR, ElevatorState.ALGAE_PROCESSOR, HandState.SCORE_ALGAE_PROCESSOR);


            /* ******** LOLLIPOP INTAKE STATES ******** */
            case PREPARE_INTAKE_LOLLIPOP, ACTIVE_INTAKE_LOLLIPOP -> armScheduler.scheduleStates(ArmState.LOLLIPOP, HandState.LOLLIPOP, ElevatorState.LOLLIPOP);


            /* ******** CLIMB STATES ******** */
            case PREPARE_CLIMB, READY_CLIMB -> armScheduler.scheduleStates(ArmState.CLIMB, HandState.CLEAR_ALGAE, ElevatorState.IDLE);
        }
    }

}
