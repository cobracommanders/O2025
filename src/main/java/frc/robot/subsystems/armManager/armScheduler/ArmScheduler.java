package frc.robot.subsystems.armManager.armScheduler;

import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmState;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandState;

public class ArmScheduler extends StateMachine<ArmSchedulerState> {
    private final Hand hand;
    private final Elevator elevator;
    private final Arm arm;

    private ArmState armState;
    private HandState handState;
    private ElevatorState elevatorState;

    public ArmScheduler(
            Hand hand,
            Elevator elevator,
            Arm arm
    ) {
        super(ArmSchedulerState.MATCH_START);
        this.hand = hand;
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    protected ArmSchedulerState getNextState(ArmSchedulerState currentState) {
        ArmSchedulerState nextState = currentState;

        switch (currentState) {
            case ARM_UP -> {
                if (arm.atGoal()) {
                    nextState = ArmSchedulerState.ELEVATOR_TO_POSITION;
                }
            }
            case ELEVATOR_TO_POSITION -> {
                if (elevator.atGoal()) {
                    nextState = ArmSchedulerState.ARM_TO_POSITION;
                }
            }
            case ARM_TO_POSITION -> {
                if (arm.atGoal()) {
                    nextState = ArmSchedulerState.READY;
                }
            }
            case READY, MATCH_START -> {
            }
        }

        return nextState;
    }

    @Override
    protected void afterTransition(ArmSchedulerState newState) {
//        switch (newState) {
//            case ARM_UP -> {
//                hand.setState(handState);
//                arm.setState(ArmState.IDLE);
//            }
//            case ELEVATOR_TO_POSITION -> {
//                elevator.setState(elevatorState);
//            }
//            case ARM_TO_POSITION -> {
//                hand.setState(handState); //needs to happen for coral mode skipping ARM_UP and ELEVATOR_TO_POS
//                arm.setState(armState);
//            }
//            case READY -> {
//                armState = null;
//                handState = null;
//                elevatorState = null;
//            }
//        }
    }

    public void scheduleStates(ArmState armState, HandState handState, ElevatorState elevatorState) {
        this.armState = armState;
        this.handState = handState;
        this.elevatorState = elevatorState;
//        if (isHandoffArmState(armState)
//                && elevator.getHeight() > ElevatorState.HANDOFF.getPosition() - Constants.ElevatorConstants.Tolerance) {
//            this.setStateFromRequest(ArmSchedulerState.ARM_TO_POSITION);
//        } else {
//            this.setStateFromRequest(ArmSchedulerState.ARM_UP);
//        }

        arm.setState(armState);
        hand.setState(handState);
        elevator.setState(elevatorState);
    }

    public boolean isHandoffArmState(ArmState armState) {
        return armState == ArmState.HANDOFF_LEFT || armState == ArmState.HANDOFF_RIGHT || armState == ArmState.HANDOFF_MIDDLE;
    }

    /**
     * Checks that the scheduler is in the READY state and both the Arm and Elevator are at position.
     */
    public boolean atPosition() {
        return /*getState() == ArmSchedulerState.READY &&*/ elevator.atGoal() && arm.atGoal();
    }
}
