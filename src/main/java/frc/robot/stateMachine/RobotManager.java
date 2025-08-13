package frc.robot.stateMachine;

import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;

public class RobotManager extends StateMachine<RobotState> {
    public final ArmManager armManager;

    public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

    public RobotManager() {
        super(RobotState.IDLE);
        this.armManager = ArmManager.getInstance();

    }

    @Override
    protected void collectInputs() {
    }

    @Override
    protected RobotState getNextState(RobotState currentState) {
        flags.log();
        RobotState nextState = currentState;
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case IDLE:
                    nextState = RobotState.PREPARE_IDLE;
                case INTAKE_CORAL:
                    nextState = RobotState.INTAKING_CORAL;
                    break;
                case CLIMB:
                    nextState = RobotState.CLIMB;
                    break;
                case L1:
                    nextState = RobotState.WAIT_L1;
                    break;
                case L4:
                    nextState = RobotState.WAIT_L4;
                    break;
                case GROUND_ALGAE_INTAKE:
                    nextState = RobotState.GROUND_ALGAE_INTAKE;
                    break;
                case HIGH_REEF_ALGAE_INTAKE:
                    nextState = RobotState.HIGH_REEF_ALGAE_INTAKE;
                    break;
                case LOW_REEF_ALGAE_INTAKE:
                    nextState = RobotState.LOW_REEF_ALGAE_INTAKE;
                    break;
                case HANDOFF:
                    nextState = RobotState.HANDOFF;
                    break;
                case SCORE:
                    switch (nextState) {
                        case WAIT_L1:
                            nextState = RobotState.SCORE_L1;
                            break;
                        case WAIT_L4:
                            nextState = RobotState.SCORE_L4;
                            break;
                        case BARGE_WAIT:
                            nextState = RobotState.BARGE_SCORE;
                            break;
                        case PROCESSOR_WAIT:
                            nextState = RobotState.PROCESSOR_SCORE;
                            break;
                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
        }

        switch (currentState) {

            case INTAKING_CORAL:
            //fill later when mereged with other subsystems
                // if () {
                //     nextState = RobotState.PREPARE_IDLE;
                // }
                break;
        }
        flags.clear();
        return nextState;
            
    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                armManager.setStateFromRequest(ArmManagerStates.IDLE);
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void intakeRequest() {
        flags.check(RobotFlag.INTAKE_CORAL);
    }

    public void scoreRequest() {
        flags.check(RobotFlag.SCORE);
    }

    public void climbRequest() {
        flags.check(RobotFlag.CLIMB);
    }

    public void L1Request() {
        flags.check(RobotFlag.L1);
    }

    public void L4Request() {
        flags.check(RobotFlag.L4);
    }

    public void idleRequest() {
        flags.check(RobotFlag.IDLE);
    }

    public void groundAlgaeRequest() {
        flags.check(RobotFlag.GROUND_ALGAE_INTAKE);
    }

    public void highReefAlgaeRequest() {
        flags.check(RobotFlag.HIGH_REEF_ALGAE_INTAKE);
    }

    public void lowReefAlgaeRequest() {
        flags.check(RobotFlag.LOW_REEF_ALGAE_INTAKE);
    }

    public void handoffRequest() {
        flags.check(RobotFlag.HANDOFF);
    }
}