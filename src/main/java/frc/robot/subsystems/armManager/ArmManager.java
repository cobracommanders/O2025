package frc.robot.subsystems.armManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmState;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandState;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

//TODO What's the difference between atGoal() and isReady()
//TODO There's a ALGAE_AWAIT_LEAVE_REEF state, but uses isReadyToReturnToIdle for coral
public class ArmManager extends StateMachine<ArmManagerState> {
    private final Hand hand;
    private final Elevator elevator;
    private final Arm arm;
    private final CoralDetector coralDetector = CoralDetector.getInstance();

    private final ArmScheduler armScheduler;

    public ArmManager(
            Hand hand,
            Elevator elevator,
            Arm arm
    ) {
        super(ArmManagerState.PREPARE_IDLE);
        this.hand = hand;
        this.elevator = elevator;
        this.arm = arm;

        this.armScheduler = new ArmScheduler(hand, elevator, arm);
    }

    public boolean isReadyToReturnToIdle() {
        //TODO Does it do something else in auto?
        if (DriverStation.isAutonomous()) return false;
        //TODO Is this just measuring distance from the target scoring pose? If so, would being 0.25m to the left/right also allow the arm to move?
        return AutoAlign.getInstance().usedScoringPose.getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 0.25;
    }

    protected ArmManagerState getNextState(ArmManagerState currentState) {
        ArmManagerState nextState = currentState;

        switch (currentState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (atGoal()) {
                    nextState = ArmManagerState.INTAKE_LOLLIPOP;
                }
            }
            case PREPARE_IDLE -> {
                if (isReady()) {
                    nextState = ArmManagerState.IDLE;
                }
            }
            case PREPARE_SCORE_L4 -> {
                if (atGoal()) {
                    nextState = ArmManagerState.WAIT_L4;
                }
            }
            case PREPARE_SCORE_L3 -> {
                if (atGoal()) {
                    nextState = ArmManagerState.WAIT_L3;
                }
            }
            case PREPARE_SCORE_L2 -> {
                if (atGoal()) {
                    nextState = ArmManagerState.WAIT_L2;
                }
            }

            case SCORE_L4, SCORE_L3, SCORE_L2 -> {
                if (isReadyToReturnToIdle()) {
                    if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.NORMAL_MODE) {
                        nextState = ArmManagerState.PREPARE_IDLE;
                    } else {
                        nextState = ArmManagerState.PREPARE_HANDOFF_CORAL_MODE;
                    }
                }
            }

            case PREPARE_HANDOFF_LEFT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerState.WAIT_HANDOFF_LEFT;
                }
            }

            case PREPARE_HANDOFF_RIGHT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerState.WAIT_HANDOFF_RIGHT;
                }
            }

            case PREPARE_HANDOFF_MIDDLE -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerState.WAIT_HANDOFF_MIDDLE;
                }
            }

            case PREPARE_HANDOFF_CORAL_MODE -> {
                if (isReady()) {
                    nextState = ArmManagerState.WAIT_HANDOFF_CORAL_MODE;
                }
            }

            case WAIT_HANDOFF_CORAL_MODE -> {
                if (GroundManager.getInstance().getState() == GroundManagerStates.WAIT_HANDOFF && coralDetector.hasCoral()) {
                    nextState = switch (coralDetector.getState()) {
                        case LEFT -> ArmManagerState.PREPARE_HANDOFF_LEFT;
                        case RIGHT -> ArmManagerState.PREPARE_HANDOFF_RIGHT;
                        case MIDDLE -> ArmManagerState.PREPARE_HANDOFF_MIDDLE;
                        // This default shouldn't be possible to return given the above coralDetector.hasCoral() check
                        default -> ArmManagerState.PREPARE_HANDOFF_MIDDLE;
                    };
                }
            }

            case PREPARE_INTAKE_GROUND_ALGAE -> {
                if (isReady()) {
                    nextState = ArmManagerState.INTAKE_GROUND_ALGAE;
                }
            }
            case INTAKE_GROUND_ALGAE -> {
                if (hand.hasAlgae()) {
                    nextState = ArmManagerState.PREPARE_IDLE;
                }
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerState.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                }
                if (isReady()) {
                    nextState = ArmManagerState.INTAKE_HIGH_REEF_ALGAE;
                }
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerState.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                } else if (isReady()) {
                    nextState = ArmManagerState.INTAKE_LOW_REEF_ALGAE;
                }
            }

            case INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L2) {
                    nextState = ArmManagerState.PREPARE_INTAKE_LOW_REEF_ALGAE;
                } else {
                    nextState = ArmManagerState.ALGAE_AWAIT_LEAVE_REEF;
                }
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerState.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                } else {
                    nextState = ArmManagerState.ALGAE_AWAIT_LEAVE_REEF;
                }
            }
            case ALGAE_AWAIT_LEAVE_REEF -> {
                if (AutoAlign.getInstance().getAlgaeDistance().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 1.25) {
                    nextState = ArmManagerState.PREPARE_IDLE;
                }
            }
            case SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR -> {
                if (timeout(1)) {
                    nextState = ArmManagerState.PREPARE_IDLE;
                }
            }

            case PREPARE_SCORE_ALGAE_NET -> {
                if (isReady()) {
                    nextState = ArmManagerState.WAIT_SCORE_ALGAE_NET;
                }
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (isReady()) {
                    nextState = ArmManagerState.WAIT_SCORE_ALGAE_PROCESSOR;
                }
            }
            case PREPARE_INVERTED_HANDOFF -> {
                if (isReady()) {
                    nextState = ArmManagerState.WAIT_INVERTED_HANDOFF;
                }
            }

            case IDLE,
                 WAIT_SCORE_ALGAE_NET,
                 WAIT_SCORE_ALGAE_PROCESSOR,
                 WAIT_L4,
                 WAIT_L3,
                 WAIT_L2,
                 WAIT_HANDOFF_RIGHT,
                 WAIT_HANDOFF_MIDDLE,
                 WAIT_HANDOFF_LEFT,
                 WAIT_INVERTED_HANDOFF,
                 INVERTED_HANDOFF,
                 CLIMB,
                 INTAKE_LOLLIPOP -> {
                // Await external control
            }
        }

        return nextState;
    }

    public void setState(ArmManagerState state) {
        if (isReady()) {
            setStateFromRequest(state);
        }
    }

    public boolean isReady() {
        return armScheduler.isReady();
    }

    @Override
    protected void afterTransition(ArmManagerState newState) {
        switch (newState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (DriverStation.isAutonomous()) {
                    arm.setState(ArmState.LOLLIPOP);
                    hand.setState(HandState.LOLLIPOP);
                    elevator.setState(ElevatorState.LOLLIPOP);
                } else {
                    armScheduler.scheduleStates(ArmState.LOLLIPOP, HandState.LOLLIPOP, ElevatorState.LOLLIPOP);
                }
            }
            case PREPARE_IDLE -> {
                armScheduler.scheduleStates(ArmState.IDLE, HandState.IDLE, ElevatorState.IDLE);
            }
            case IDLE -> {
                if (timeout(1)) {
                    arm.syncEncoder();
                }
            }
            case PREPARE_INTAKE_GROUND_ALGAE -> {
                armScheduler.scheduleStates(ArmState.INTAKE_GROUND_ALGAE, HandState.INTAKE_GROUND_ALGAE,
                        ElevatorState.GROUND_ALGAE);
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmState.INTAKE_HIGH_REEF_ALGAE, HandState.INTAKE_HIGH_REEF_ALGAE,
                        ElevatorState.HIGH_REEF_ALGAE);
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmState.INTAKE_LOW_REEF_ALGAE, HandState.INTAKE_LOW_REEF_ALGAE,
                        ElevatorState.LOW_REEF_ALGAE);
            }

            case PREPARE_SCORE_ALGAE_NET -> {
                armScheduler.scheduleStates(ArmState.ALGAE_NET, HandState.IDLE, ElevatorState.ALGAE_NET);
            }
            case SCORE_ALGAE_NET -> {
                hand.setState(HandState.SCORE_ALGAE_NET);
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                armScheduler.scheduleStates(ArmState.ALGAE_PROCESSOR, HandState.IDLE, ElevatorState.ALGAE_PROCESSOR);
            }
            case SCORE_ALGAE_PROCESSOR -> {
                hand.setState(HandState.SCORE_ALGAE_PROCESSOR);
            }
            case PREPARE_SCORE_L4 -> {
                arm.setState(ArmState.L4);
                hand.setState(HandState.CORAL_IDLE);
                elevator.setState(ElevatorState.L4);
            }
            case SCORE_L4 -> {
                hand.setState(HandState.SCORE_CORAL);
                elevator.setState(ElevatorState.SCORE_L4);
                arm.setState(ArmState.SCORE_L4);
            }
            case PREPARE_SCORE_L3 -> {
                armScheduler.scheduleStates(ArmState.L3, HandState.CORAL_IDLE, ElevatorState.L3);
            }

            case SCORE_L3 -> {
                hand.setState(HandState.SCORE_CORAL);
                elevator.setState(ElevatorState.SCORE_L3);
                arm.setState(ArmState.SCORE_L3);
            }
            case PREPARE_SCORE_L2 -> {
                armScheduler.scheduleStates(ArmState.L2, HandState.CORAL_IDLE, ElevatorState.L2);
            }
            case SCORE_L2 -> {
                hand.setState(HandState.SCORE_CORAL);
                elevator.setState(ElevatorState.SCORE_L2);
                arm.setState(ArmState.SCORE_L2);
            }

            case PREPARE_HANDOFF_RIGHT -> {
                arm.setState(ArmState.HANDOFF_RIGHT);
                elevator.setState(ElevatorState.HANDOFF);
                hand.setState(HandState.HANDOFF);
            }
            case PREPARE_HANDOFF_MIDDLE -> {
                arm.setState(ArmState.HANDOFF_MIDDLE);
                elevator.setState(ElevatorState.HANDOFF);
                hand.setState(HandState.HANDOFF);
            }
            case PREPARE_HANDOFF_LEFT -> {
                arm.setState(ArmState.HANDOFF_LEFT);
                elevator.setState(ElevatorState.HANDOFF);
                hand.setState(HandState.HANDOFF);
            }
            case PREPARE_HANDOFF_CORAL_MODE -> {
                armScheduler.scheduleStates(ArmState.HANDOFF_MIDDLE, HandState.HANDOFF, ElevatorState.HANDOFF_CORAL_MODE);
            }

            case PREPARE_INVERTED_HANDOFF -> {
                armScheduler.scheduleStates(ArmState.HANDOFF_MIDDLE, HandState.IDLE, ElevatorState.HANDOFF);
            }
            case INVERTED_HANDOFF -> {
                armScheduler.scheduleStates(ArmState.HANDOFF_MIDDLE, HandState.INVERTED_HANDOFF, ElevatorState.HANDOFF);
            }

            case CLIMB -> armScheduler.scheduleStates(ArmState.CLIMB, HandState.IDLE, ElevatorState.IDLE);

            case INTAKE_GROUND_ALGAE,
                 INTAKE_LOLLIPOP,
                 INTAKE_HIGH_REEF_ALGAE,
                 INTAKE_LOW_REEF_ALGAE,
                 ALGAE_AWAIT_LEAVE_REEF,
                 WAIT_SCORE_ALGAE_NET,
                 WAIT_HANDOFF_LEFT,
                 WAIT_SCORE_ALGAE_PROCESSOR,
                 WAIT_HANDOFF_MIDDLE,
                 WAIT_HANDOFF_RIGHT,
                 WAIT_HANDOFF_CORAL_MODE,
                 WAIT_INVERTED_HANDOFF,
                 WAIT_L3,
                 WAIT_L2,
                 WAIT_L4 -> {
            }
        }
    }

    public Command awaitFinishedScoringCommand() {
        return Commands.waitUntil(this::finishedScoring);
    }

    private boolean finishedScoring() {
        return isScoring() && atGoal();
    }

    private boolean isScoring() {
        return getState() == ArmManagerState.SCORE_L2 || getState() == ArmManagerState.SCORE_L3 || getState() == ArmManagerState.SCORE_L4;
    }

    private boolean atGoal() {
        return arm.atGoal() && elevator.atGoal();
    }

    public void elevatorTickUp() {
        elevator.tickUp();
    }

    public void elevatorTickDown() {
        elevator.tickDown();
    }
}
