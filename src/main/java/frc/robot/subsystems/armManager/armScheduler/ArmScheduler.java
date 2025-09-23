package frc.robot.subsystems.armManager.armScheduler;

import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmStates;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorPositions;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandStates;

public class ArmScheduler extends StateMachine<ArmSchedulerStates> {
    private final Hand hand = Hand.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Arm arm = Arm.getInstance();

    private ArmStates armState;
    private HandStates handState;
    private ElevatorStates elevatorState;

    private ArmScheduler() {
        super(ArmSchedulerStates.MATCH_START);
    }

    @Override
    protected ArmSchedulerStates getNextState(ArmSchedulerStates currentState) {
        ArmSchedulerStates nextState = currentState;

        switch (currentState) {
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
            case READY, MATCH_START -> {}
        }

        return nextState;
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
                hand.setState(handState); //needs to happen for coral mode skipping ARM_UP and ELEVATOR_TO_POS
                arm.setState(armState);
            }
            case READY -> {
                armState = null;
                handState = null;
                elevatorState = null;
            }
        }
    }

    public void scheduleStates(ArmStates armState, HandStates handState, ElevatorStates elevatorState) {
        this.armState = armState;
        this.handState = handState;
        this.elevatorState = elevatorState;
        if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.CORAL_MODE
                && isArmManagerPrepareHandoff(ArmManager.getInstance().getState())
                && isHandoffArmState(armState)
                && elevator.getHeight() > ElevatorPositions.HANDOFF - elevator.tolerance) {
            this.setStateFromRequest(ArmSchedulerStates.ARM_TO_POSITION);
        } else {
            this.setStateFromRequest(ArmSchedulerStates.ARM_UP);
        }
    }

    public boolean isHandoffArmState(ArmStates armState) {
        return armState == ArmStates.HANDOFF_LEFT || armState == ArmStates.HANDOFF_RIGHT || armState == ArmStates.HANDOFF_MIDDLE;
    }

    public boolean isArmManagerPrepareHandoff(ArmManagerStates armManagerState) {
        return armManagerState == ArmManagerStates.PREPARE_HANDOFF_LEFT || armManagerState == ArmManagerStates.PREPARE_HANDOFF_MIDDLE || armManagerState == ArmManagerStates.PREPARE_HANDOFF_RIGHT || armManagerState == ArmManagerStates.WAIT_HANDOFF_MIDDLE;
    }

    public boolean isReady() {
        return getState() == ArmSchedulerStates.READY;
    }

    private static ArmScheduler instance;

    public static ArmScheduler getInstance() {
        if (instance == null)
            instance = new ArmScheduler(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
