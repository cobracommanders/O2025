package frc.robot.stateMachine;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;

public class RobotManager extends StateMachine<RobotState> {

    public final ArmManager armManager;
    public final GroundManager groundManager;
    public final Climber climber;

    public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

    public RobotManager() {
        super(RobotState.IDLE);

        this.armManager = ArmManager.getInstance();
        this.groundManager = GroundManager.getInstance();
        this.climber = Climber.getInstance();
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
                    nextState = RobotState.IDLE;
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

            case WAIT_L1, WAIT_L4, BARGE_WAIT, PROCESSOR_WAIT, HANDOFF, CLIMB, IDLE:
                // These states do not transition automatically
                break;

            case INTAKING_CORAL:
                if (groundManager.getState() == GroundManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case PREPARE_HANDOFF:
                if ((armManager.getState() == ArmManagerStates.PREPARE_HANDOFF_LEFT ||
                        armManager.getState() == ArmManagerStates.PREPARE_HANDOFF_MIDDLE ||
                        armManager.getState() == ArmManagerStates.PREPARE_HANDOFF_RIGHT) &&
                        groundManager.getState() == GroundManagerStates.HANDOFF) {
                    nextState = RobotState.HANDOFF;
                }
                break;
            case GROUND_ALGAE_INTAKE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case HIGH_REEF_ALGAE_INTAKE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case LOW_REEF_ALGAE_INTAKE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case SCORE_L1:
                if (groundManager.getState() == GroundManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case SCORE_L4:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case BARGE_SCORE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case PROCESSOR_SCORE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                    break;
                }
                flags.clear();
        }

        return nextState;

    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            case IDLE, BARGE_WAIT, PROCESSOR_WAIT, HANDOFF -> {
            }
            case SCORE_L1 -> {
                groundManager.setStateFromRequest(GroundManagerStates.SCORE_L1);
            }
            case SCORE_L4 -> {
                armManager.setState(ArmManagerStates.SCORE_L4);
            }
            case PROCESSOR_SCORE -> {
                armManager.setState(ArmManagerStates.SCORE_ALGAE_PROCESSOR);
            }
            case BARGE_SCORE -> {
                armManager.setState(ArmManagerStates.SCORE_ALGAE_NET);
            }
            case PREPARE_HANDOFF -> {
                armManager.setState(ArmManagerStates.PREPARE_HANDOFF_LEFT);
            }
            case HIGH_REEF_ALGAE_INTAKE -> {
                armManager.setState(ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE);
            }
            case LOW_REEF_ALGAE_INTAKE -> {
                armManager.setState(ArmManagerStates.PREPARE_INTAKE_LOW_REEF_ALGAE);
            }
            case GROUND_ALGAE_INTAKE -> {
                armManager.setState(ArmManagerStates.PREPARE_INTAKE_GROUND_ALGAE);
            }
            case INTAKING_CORAL -> {
                groundManager.setStateFromRequest(GroundManagerStates.PREPARE_INTAKE);
            }
            case WAIT_L1 -> {
                groundManager.setStateFromRequest(GroundManagerStates.PREPARE_SCORE_L1);
            }
            case WAIT_L4 -> {
                armManager.setState(ArmManagerStates.PREPARE_SCORE_L4);
            }
            case CLIMB -> {

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
