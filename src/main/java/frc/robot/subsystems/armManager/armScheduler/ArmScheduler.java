package frc.robot.subsystems.armManager.armScheduler;

import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmStates;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandStates;

public class ArmScheduler extends StateMachine<ArmSchedulerStates> {

    public final Hand hand;
    public final Elevator elevator;
    public final Arm arm;

    private ArmStates armState;
    private HandStates handState;
    private ElevatorStates elevatorState;

    private boolean begin = false;

    public ArmScheduler() {
        super(ArmSchedulerStates.START);

        this.arm = Arm.getInstance();
        this.elevator = Elevator.getInstance();
        this.hand = Hand.getInstance();
    }

    @Override
    protected ArmSchedulerStates getNextState(ArmSchedulerStates currentState) {
        ArmSchedulerStates nextState = currentState;

        switch (currentState) {
            case START -> {
                if (begin) {
                    begin = false;
                    nextState = ArmSchedulerStates.ARM_UP;
                }
            }
            case ARM_UP -> {
                if (arm.atGoal()) {
                    nextState = ArmSchedulerStates.ELEVATOR_TO_POSITION;
                }
            }
            case ELEVATOR_TO_POSITION -> {
                if (elevator.atGoal()) {
                    nextState = ArmSchedulerStates.ARM_TO_POSITION;
                }
            }
            case ARM_TO_POSITION -> {
                if (arm.atGoal()) {
                    nextState = ArmSchedulerStates.READY;
                }
            }
            case READY -> {
                if (begin) {
                    begin = false;
                    nextState = ArmSchedulerStates.ARM_UP;
                }
            }
        }

        return nextState;
    }

    public void scheduleStates(ArmStates armState, HandStates handState, ElevatorStates elevatorState) {
        this.armState = armState;
        this.handState = handState;
        this.elevatorState = elevatorState;
        this.setStateFromRequest(ArmSchedulerStates.ARM_UP);
    }

    @Override
    protected void afterTransition(ArmSchedulerStates newState) {
        switch (newState) {
            case ARM_UP -> {
                hand.setState(handState);
                arm.setState(ArmStates.IDLE);
            }
            case ELEVATOR_TO_POSITION -> {
                elevator.setState(elevatorState);
            }
            case ARM_TO_POSITION -> {
                arm.setState(armState);
            }
            case READY -> {
                armState = null;
                handState = null;
                elevatorState = null;
            }
        }
    }

    public boolean isReady(){
        return getState() == ArmSchedulerStates.READY;
    }

    private static ArmScheduler instance;

    public static ArmScheduler getInstance() {
        if (instance == null)
            instance = new ArmScheduler(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
