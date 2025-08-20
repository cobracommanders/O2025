package frc.robot.subsystems.armManager;

import dev.doglog.DogLog;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmStates;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.armScheduler.ArmSchedulerStates;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandStates;

public class ArmManager extends StateMachine<ArmManagerStates> {
    private final String name = getName();
    public final Hand hand;
    public final Elevator elevator;
    public final Arm arm;

    private boolean synced = false;

    public final ArmScheduler armScheduler;

    public ArmManager() {
        super(ArmManagerStates.PREPARE_IDLE);

        this.hand = Hand.getInstance();
        this.elevator = Elevator.getInstance();
        this.arm = Arm.getInstance();
        this.armScheduler = ArmScheduler.getInstance();
    }

    protected ArmManagerStates getNextState(ArmManagerStates currentState) {
        ArmManagerStates nextState = currentState;

        switch (currentState) {
            case PREPARE_IDLE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.IDLE;
                }
            }

            case SCORE_L4 -> {
                if(timeout(3)){
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }

            case PREPARE_HANDOFF_LEFT -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_LEFT;
                }
            }

            case PREPARE_HANDOFF_RIGHT -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_RIGHT;
                }
            }

            case PREPARE_HANDOFF_MIDDLE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_MIDDLE;
                }
            }

            case PREPARE_INTAKE_GROUND_ALGAE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_GROUND_ALGAE;
                }
            }
            case INTAKE_GROUND_ALGAE -> {
                if (hand.hasAlgae()) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_HIGH_REEF_ALGAE;
                }
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                if (hand.hasAlgae()){
                    nextState = ArmManagerStates.ALGAE_LEAVE_REEF;
                }
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_LOW_REEF_ALGAE;
                }
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                if (hand.hasAlgae() && (timeout(2))){
                    nextState = ArmManagerStates.ALGAE_LEAVE_REEF;
                }
            }
            case ALGAE_LEAVE_REEF -> {
                if(timeout(2)) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_SCORE_ALGAE_NET -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_NET;
                }
            }
            case WAIT_SCORE_ALGAE_NET -> {
            }
            case SCORE_ALGAE_NET -> {
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_PROCESSOR;
                }
            }
            case WAIT_SCORE_ALGAE_PROCESSOR -> {
            }
            case SCORE_ALGAE_PROCESSOR -> {
            }

        }

        return nextState;
    }

    public void setState(ArmManagerStates state) {
        if (armScheduler.isReady()) {
            setStateFromRequest(state);
        }else{
            //do nothing
        }
    }

    @Override
    protected void afterTransition(ArmManagerStates newState) {
        synced = false;
        switch (newState) {
            case PREPARE_IDLE -> {
                armScheduler.scheduleStates(ArmStates.IDLE, HandStates.IDLE, ElevatorStates.IDLE);
            }
            case IDLE -> {
                if (!synced && timeout(1)) {
                    arm.syncEncoder();
                    synced = true;
                }
            }
            case PREPARE_INTAKE_GROUND_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_GROUND_ALGAE, HandStates.INTAKE_GROUND_ALGAE,
                        ElevatorStates.GROUND_ALGAE);
            }
            case INTAKE_GROUND_ALGAE -> {
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_HIGH_REEF_ALGAE, HandStates.INTAKE_HIGH_REEF_ALGAE,
                        ElevatorStates.HIGH_REEF_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_LOW_REEF_ALGAE, HandStates.INTAKE_LOW_REEF_ALGAE,
                        ElevatorStates.LOW_REEF_ALGAE);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
            }
            case PREPARE_SCORE_ALGAE_NET -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_NET, HandStates.IDLE, ElevatorStates.ALGAE_NET);
            }
            case WAIT_SCORE_ALGAE_NET -> {
            }
            case SCORE_ALGAE_NET -> {
                hand.setState(HandStates.SCORE_ALGAE_NET);
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_PROCESSOR, HandStates.IDLE, ElevatorStates.ALGAE_PROCESSOR);
            }
            case WAIT_SCORE_ALGAE_PROCESSOR -> {
            }
            case SCORE_ALGAE_PROCESSOR -> {
                hand.setState(HandStates.SCORE_ALGAE_PROCESSOR);
            }
            case PREPARE_SCORE_L4 -> {
                armScheduler.scheduleStates(ArmStates.SCORE_L4, HandStates.CORAL_IDLE, ElevatorStates.L4);
            }
            case WAIT_L4 -> {
                armScheduler.scheduleStates(ArmStates.L4, HandStates.CORAL_IDLE, ElevatorStates.L4);
            }
            case SCORE_L4 -> {
                hand.setState(HandStates.CORAL_IDLE);
                elevator.setState(ElevatorStates.SCORE_L4);
            }
            case PREPARE_HANDOFF_RIGHT -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_RIGHT, HandStates.HANDOFF, ElevatorStates.HANDOFF);
            }
            case WAIT_HANDOFF_RIGHT -> {
            }
            case PREPARE_HANDOFF_MIDDLE -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.HANDOFF, ElevatorStates.HANDOFF);
            }
            case WAIT_HANDOFF_MIDDLE -> {
            }
            case PREPARE_HANDOFF_LEFT -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_LEFT, HandStates.HANDOFF, ElevatorStates.HANDOFF);
            }
            case WAIT_HANDOFF_LEFT -> {
            }
            case CLIMB -> {
                armScheduler.scheduleStates(ArmStates.IDLE, HandStates.IDLE, ElevatorStates.IDLE);
            }

        }
    }

    private static ArmManager instance;

    public static ArmManager getInstance() {
        if (instance == null)
            instance = new ArmManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
