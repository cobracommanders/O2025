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
import frc.robot.subsystems.armManager.arm.ArmStates;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandStates;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

//TODO What's the difference between atGoal() and isReady()
//TODO There's a ALGAE_AWAIT_LEAVE_REEF state, but uses isReadyToReturnToIdle for coral
public class ArmManager extends StateMachine<ArmManagerStates> {
    private final Hand hand = Hand.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Arm arm = Arm.getInstance();
    private final CoralDetector coralDetector = CoralDetector.getInstance();

    public final ArmScheduler armScheduler = ArmScheduler.getInstance();

    private ArmManager() {
        super(ArmManagerStates.PREPARE_IDLE);
    }

    public boolean isReadyToReturnToIdle() {
        //TODO Does it do something else in auto?
        if (DriverStation.isAutonomous()) return false;
        //TODO Is this just measuring distance from the target scoring pose? If so, would being 0.25m to the left/right also allow the arm to move?
        return AutoAlign.getInstance().usedScoringPose.getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 0.25;
    }

    protected ArmManagerStates getNextState(ArmManagerStates currentState) {
        ArmManagerStates nextState = currentState;

        switch (currentState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (atGoal()) {
                    nextState = ArmManagerStates.INTAKE_LOLLIPOP;
                }
            }
            case PREPARE_IDLE -> {
                if (isReady()) {
                    nextState = ArmManagerStates.IDLE;
                }
            }
            case PREPARE_SCORE_L4 -> {
                if (atGoal()) {
                    nextState = ArmManagerStates.WAIT_L4;
                }
            }
            case PREPARE_SCORE_L3 -> {
                if (atGoal()) {
                    nextState = ArmManagerStates.WAIT_L3;
                }
            }
            case PREPARE_SCORE_L2 -> {
                if (atGoal()) {
                    nextState = ArmManagerStates.WAIT_L2;
                }
            }

            case SCORE_L4, SCORE_L3, SCORE_L2 -> {
                if (isReadyToReturnToIdle()) {
                    if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.NORMAL_MODE) {
                        nextState = ArmManagerStates.PREPARE_IDLE;
                    } else {
                        nextState = ArmManagerStates.PREPARE_HANDOFF_CORAL_MODE;
                    }
                }
            }

            case PREPARE_HANDOFF_LEFT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_LEFT;
                }
            }

            case PREPARE_HANDOFF_RIGHT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_RIGHT;
                }
            }

            case PREPARE_HANDOFF_MIDDLE -> {
                if (arm.atGoal() && elevator.atGoal()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_MIDDLE;
                }
            }

            case PREPARE_HANDOFF_CORAL_MODE -> {
                if (isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_CORAL_MODE;
                }
            }

            case WAIT_HANDOFF_CORAL_MODE -> {
                if (GroundManager.getInstance().getState() == GroundManagerStates.WAIT_HANDOFF && coralDetector.hasCoral()) {
                    nextState = switch (coralDetector.getState()) {
                        case LEFT -> ArmManagerStates.PREPARE_HANDOFF_LEFT;
                        case RIGHT -> ArmManagerStates.PREPARE_HANDOFF_RIGHT;
                        case MIDDLE -> ArmManagerStates.PREPARE_HANDOFF_MIDDLE;
                        // This default shouldn't be possible to return given the above coralDetector.hasCoral() check
                        default -> ArmManagerStates.PREPARE_HANDOFF_MIDDLE;
                    };
                }
            }

            case PREPARE_INTAKE_GROUND_ALGAE -> {
                if (isReady()) {
                    nextState = ArmManagerStates.INTAKE_GROUND_ALGAE;
                }
            }
            case INTAKE_GROUND_ALGAE -> {
                if (hand.hasAlgae()) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                }
                if (isReady()) {
                    nextState = ArmManagerStates.INTAKE_HIGH_REEF_ALGAE;
                }
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                } else if (isReady()) {
                    nextState = ArmManagerStates.INTAKE_LOW_REEF_ALGAE;
                }
            }

            case INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L2) {
                    nextState = ArmManagerStates.PREPARE_INTAKE_LOW_REEF_ALGAE;
                } else {
                    nextState = ArmManagerStates.ALGAE_AWAIT_LEAVE_REEF;
                }
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                    nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                } else {
                    nextState = ArmManagerStates.ALGAE_AWAIT_LEAVE_REEF;
                }
            }
            case ALGAE_AWAIT_LEAVE_REEF -> {
                if (AutoAlign.getInstance().getAlgaeDistance().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 1.25) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR -> {
                if (timeout(1)) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }

            case PREPARE_SCORE_ALGAE_NET -> {
                if (isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_NET;
                }
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_PROCESSOR;
                }
            }
            case PREPARE_INVERTED_HANDOFF -> {
                if (isReady()) {
                    nextState = ArmManagerStates.WAIT_INVERTED_HANDOFF;
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
                 CLIMB,
                 INTAKE_LOLLIPOP -> {
                // Await external control
            }
        }

        return nextState;
    }

    public void setState(ArmManagerStates state) {
        if (isReady()) {
            setStateFromRequest(state);
        }
    }

    public boolean isReady() {
        return armScheduler.isReady();
    }

    @Override
    protected void afterTransition(ArmManagerStates newState) {
        switch (newState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (DriverStation.isAutonomous()) {
                    arm.setState(ArmStates.LOLLIPOP);
                    hand.setState(HandStates.LOLLIPOP);
                    elevator.setState(ElevatorStates.LOLLIPOP);
                } else {
                    armScheduler.scheduleStates(ArmStates.LOLLIPOP, HandStates.LOLLIPOP, ElevatorStates.LOLLIPOP);
                }
            }
            case PREPARE_IDLE -> {
                armScheduler.scheduleStates(ArmStates.IDLE, HandStates.IDLE, ElevatorStates.IDLE);
            }
            case IDLE -> {
                if (timeout(1)) {
                    arm.syncEncoder();
                }
            }
            case PREPARE_INTAKE_GROUND_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_GROUND_ALGAE, HandStates.INTAKE_GROUND_ALGAE,
                        ElevatorStates.GROUND_ALGAE);
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_HIGH_REEF_ALGAE, HandStates.INTAKE_HIGH_REEF_ALGAE,
                        ElevatorStates.HIGH_REEF_ALGAE);
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_LOW_REEF_ALGAE, HandStates.INTAKE_LOW_REEF_ALGAE,
                        ElevatorStates.LOW_REEF_ALGAE);
            }

            case PREPARE_SCORE_ALGAE_NET -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_NET, HandStates.IDLE, ElevatorStates.ALGAE_NET);
            }
            case SCORE_ALGAE_NET -> {
                hand.setState(HandStates.SCORE_ALGAE_NET);
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_PROCESSOR, HandStates.IDLE, ElevatorStates.ALGAE_PROCESSOR);
            }
            case SCORE_ALGAE_PROCESSOR -> {
                hand.setState(HandStates.SCORE_ALGAE_PROCESSOR);
            }
            case PREPARE_SCORE_L4 -> {
                arm.setState(ArmStates.L4);
                hand.setState(HandStates.CORAL_IDLE);
                elevator.setState(ElevatorStates.L4);
            }
            case SCORE_L4 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L4);
                arm.setState(ArmStates.SCORE_L4);
            }
            case PREPARE_SCORE_L3 -> {
                armScheduler.scheduleStates(ArmStates.L3, HandStates.CORAL_IDLE, ElevatorStates.L3);
            }

            case SCORE_L3 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L3);
                arm.setState(ArmStates.SCORE_L3);
            }
            case PREPARE_SCORE_L2 -> {
                armScheduler.scheduleStates(ArmStates.L2, HandStates.CORAL_IDLE, ElevatorStates.L2);
            }
            case SCORE_L2 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L2);
                arm.setState(ArmStates.SCORE_L2);
            }

            case PREPARE_HANDOFF_RIGHT -> {
                arm.setState(ArmStates.HANDOFF_RIGHT);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
            }
            case PREPARE_HANDOFF_MIDDLE -> {
                arm.setState(ArmStates.HANDOFF_MIDDLE);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
            }
            case PREPARE_HANDOFF_LEFT -> {
                arm.setState(ArmStates.HANDOFF_LEFT);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
            }
            case PREPARE_HANDOFF_CORAL_MODE -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.HANDOFF, ElevatorStates.HANDOFF_CORAL_MODE);
            }

            case PREPARE_INVERTED_HANDOFF -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.IDLE, ElevatorStates.HANDOFF);
            }
            case CLIMB -> armScheduler.scheduleStates(ArmStates.CLIMB, HandStates.IDLE, ElevatorStates.IDLE);

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
        return getState() == ArmManagerStates.SCORE_L2 || getState() == ArmManagerStates.SCORE_L3 || getState() == ArmManagerStates.SCORE_L4;
    }

    private boolean atGoal() {
        return arm.atGoal() && elevator.atGoal();
    }

    private static ArmManager instance;

    public static ArmManager getInstance() {
        if (instance == null)
            instance = new ArmManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
