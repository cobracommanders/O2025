package frc.robot.stateMachine;

public class RobotManager extends StateMachine<RobotState> {

    public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

    public RobotManager() {
        super(RobotState.IDLE);

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
            case PREPARE_IDLE:
                break;
            case PREPARE_HANDOFF:
                break;
            case GROUND_ALGAE_INTAKE:
                break;
            case HIGH_REEF_ALGAE_INTAKE:
                break;
            case LOW_REEF_ALGAE_INTAKE:
                break;
            case SCORE_L1:
                // add automatic transition???
                break;
            case SCORE_L4:
                // add automatic transition???
                break;
            case BARGE_SCORE:
                // add automatic transition???
                break;
            case PROCESSOR_SCORE:
                // add automatic transition???
                break;

        }
        flags.clear();
        return nextState;
            
    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            case IDLE, WAIT_L1, WAIT_L4, BARGE_WAIT, PROCESSOR_WAIT -> {

            }
            case SCORE_L1 -> {

            }
            case SCORE_L4 -> {

            }
            case PROCESSOR_SCORE -> {

            }
            case BARGE_SCORE -> {

            }
            case PREPARE_IDLE -> {
                //fill after merge to main
            }
            case PREPARE_HANDOFF -> {
                //fill after merge to main
            }
            case HIGH_REEF_ALGAE_INTAKE -> {

            }
            case LOW_REEF_ALGAE_INTAKE -> {

            }
            case GROUND_ALGAE_INTAKE -> {

            }
            case INTAKING_CORAL -> {

            }
            case CLIMB -> {

            }
            case HANDOFF -> {

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
