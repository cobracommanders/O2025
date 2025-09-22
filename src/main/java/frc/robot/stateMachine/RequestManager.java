package frc.robot.stateMachine;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.stateMachine.OperatorOptions.CoralMode;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

public class RequestManager extends StateMachine<RequestManagerStates> {
    private final ArmManager armManager = ArmManager.getInstance();
    private final Hand hand = Hand.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final GroundManager groundManager = GroundManager.getInstance();
    private final Climber climber = Climber.getInstance();
    private final CoralDetector coralDetector = CoralDetector.getInstance();
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    private final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);
    private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

    private RequestManager() {
        super(RequestManagerStates.INDEPENDENT);
    }

    @Override
    protected void collectInputs() {
        driveSubsystem.setElevatorHeight(elevator.getHeight()); // TODO Does this have to do with request managing?
        DogLog.log("coral mode", operatorOptions.coralMode);
    }

    @Override
    protected RequestManagerStates getNextState(RequestManagerStates currentState) {
        flags.log();
        RequestManagerStates nextState = currentState;
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case CLIMB:
                    if (armManager.isReady()) {
                        nextState = RequestManagerStates.CLIMB;
                        flags.remove(flag);
                    }
                    break;
                case HANDOFF:
                    if (armManager.isReady()) {
                        nextState = RequestManagerStates.PREPARE_HANDOFF;
                        flags.remove(flag);
                    }
                    break;
                case INVERTED_HANDOFF:
                    if (armManager.isReady()) {
                        nextState = RequestManagerStates.PREPARE_INVERTED_HANDOFF;
                        flags.remove(flag);
                    }
                    break;
                case RESET_TO_IDLE:
                    if (armManager.isReady()) {
                        nextState = RequestManagerStates.PREPARE_IDLE;
                        flags.remove(flag);
                    }
                    break;
                default:
                    break;
            }
        }

        switch (currentState) {
            case PREPARE_HANDOFF:
                if ((armManager.getState() == ArmManagerStates.WAIT_HANDOFF_LEFT ||
                        armManager.getState() == ArmManagerStates.WAIT_HANDOFF_MIDDLE ||
                        armManager.getState() == ArmManagerStates.WAIT_HANDOFF_RIGHT) &&
                        groundManager.getState() == GroundManagerStates.WAIT_HANDOFF) {
                    nextState = RequestManagerStates.HANDOFF;
                }
                break;
            case HANDOFF:
                if ((operatorOptions.coralMode == CoralMode.NORMAL_MODE && timeout(0.1)) || timeout(0.2)) {
                    nextState = RequestManagerStates.INDEPENDENT;

                }
                break;
            case PREPARE_INVERTED_HANDOFF:
                if ((armManager.getState() == ArmManagerStates.WAIT_INVERTED_HANDOFF) &&
                        groundManager.getState() == GroundManagerStates.WAIT_INVERTED_HANDOFF) {
                    nextState = RequestManagerStates.INVERTED_HANDOFF;
                }
                break;
            case INVERTED_HANDOFF:
                if (coralDetector.hasCoral() || timeout(0.5)) {
                    nextState = RequestManagerStates.INDEPENDENT;
                }
                break;
            case PREPARE_IDLE:
                if ((operatorOptions.coralMode == CoralMode.NORMAL_MODE) &&
                        ((armManager.getState() == ArmManagerStates.IDLE) &&
                                groundManager.getState() == GroundManagerStates.IDLE)) {
                    nextState = RequestManagerStates.INDEPENDENT;
                } else {
                    //we are in coral mode
                    if (armManager.getState() == ArmManagerStates.WAIT_HANDOFF_MIDDLE && groundManager.getState() == GroundManagerStates.IDLE) {
                        nextState = RequestManagerStates.INDEPENDENT;
                    }
                }


                break;
            case CLIMB:
            case INDEPENDENT:
                break;
        }
        return nextState;
    }

    @Override
    protected void afterTransition(RequestManagerStates newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                if (operatorOptions.coralMode == CoralMode.CORAL_MODE) {
                    groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_MIDDLE);
                } else if (operatorOptions.coralMode == CoralMode.NORMAL_MODE) {
                    groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                    armManager.setState(ArmManagerStates.PREPARE_IDLE);
                }

            }
            case HANDOFF -> {
                groundManager.setState(GroundManagerStates.HANDOFF);
            }
            case INDEPENDENT -> {
            }
            case PREPARE_HANDOFF -> {
                // need to figure out which handoff we are doing...
                if (armManager.getState() != ArmManagerStates.WAIT_HANDOFF_CORAL_MODE) {
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_CORAL_MODE);
                }
                groundManager.setState(GroundManagerStates.PREPARE_HANDOFF);
            }
            case PREPARE_INVERTED_HANDOFF -> {
                armManager.setState(ArmManagerStates.PREPARE_INVERTED_HANDOFF);
                groundManager.setState(GroundManagerStates.PREPARE_INVERTED_HANDOFF);
            }
            case INVERTED_HANDOFF -> {
                hand.setState(HandStates.INVERTED_HANDOFF);
            }
            case CLIMB -> {
                climber.setState(ClimberStates.DEPLOYING);
                armManager.setState(ArmManagerStates.CLIMB);
                groundManager.setState(GroundManagerStates.CLIMB);
            }
        }

    }

    @Override
    public void periodic() {
        super.periodic();
        if (getState() == RequestManagerStates.INDEPENDENT) {
            sendManagerRequests();
        }
    }

    private void sendManagerRequests() {
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case INTAKE_ALGAE:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        switch (operatorOptions.algaeIntakeLevel) {
                            case GROUND_ALGAE:
                                armManager.setState(ArmManagerStates.PREPARE_INTAKE_GROUND_ALGAE);
                                break;
                            case HIGH_REEF:
                                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L2) {
                                    armManager.setState(ArmManagerStates.PREPARE_INTAKE_LOW_REEF_ALGAE);
                                }
                                armManager.setState(ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE);
                                break;
                            case LOW_REEF:
                                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                                    armManager.setState(ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE);
                                }
                                armManager.setState(ArmManagerStates.PREPARE_INTAKE_LOW_REEF_ALGAE);
                                break;
                        }
                    }
                    break;
                case INTAKE_CORAL_LOLLIPOP:
                    flags.remove(flag);
                    if (DriverStation.isAutonomous()) {
                        armManager.setState(ArmManagerStates.PREPARE_INTAKE_LOLLIPOP);
                    }
                    break;
                case INTAKE_CORAL:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        groundManager.setState(GroundManagerStates.PREPARE_INTAKE);
                    }
                    break;
                case PREPARE_SCORE_ARM:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        switch (operatorOptions.scoreLocation) {
                            case L2:
                                armManager.setState(ArmManagerStates.PREPARE_SCORE_L2);
                                break;
                            case L3:
                                armManager.setState(ArmManagerStates.PREPARE_SCORE_L3);
                                break;
                            case L4:
                                armManager.setState(ArmManagerStates.PREPARE_SCORE_L4);
                                break;
                            case BARGE:
                                armManager.setState(ArmManagerStates.PREPARE_SCORE_ALGAE_NET);
                                break;
                            case PROCESSOR:
                                armManager.setState(ArmManagerStates.PREPARE_SCORE_ALGAE_PROCESSOR);
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                case PREPARE_SCORE_GROUND:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        switch (operatorOptions.scoreLocation) {
                            case L1:
                                groundManager.setState(GroundManagerStates.PREPARE_SCORE_L1);
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                case SCORE_ARM:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        switch (operatorOptions.scoreLocation) {
                            case L2:
                                armManager.setState(ArmManagerStates.SCORE_L2);
                                break;
                            case L3:
                                armManager.setState(ArmManagerStates.SCORE_L3);
                                break;
                            case L4:
                                armManager.setState(ArmManagerStates.SCORE_L4);
                                break;
                            case BARGE:
                                armManager.setState(ArmManagerStates.SCORE_ALGAE_NET);
                                break;
                            case PROCESSOR:
                                armManager.setState(ArmManagerStates.SCORE_ALGAE_PROCESSOR);
                                break;

                            default:
                                break;
                        }
                    }

                    break;
                case SCORE_GROUND:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        switch (operatorOptions.scoreLocation) {
                            case L1:
                                    groundManager.setState(GroundManagerStates.SCORE_L1);
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                case GROUND_IDLE:
                    flags.remove(flag);
                    if (getState() != RequestManagerStates.INDEPENDENT) {
                        // do nothing
                    } else {
                        groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                    }
                default:
                    break;
            }
        }
    }

    public void coralIntakeRequest() {
        flags.check(RobotFlag.INTAKE_CORAL);
    }

    public void lollipopIntakeRequest() {
        flags.check(RobotFlag.INTAKE_CORAL_LOLLIPOP);
    }

    public void scoreRequest() {
        if (operatorOptions.scoreLocation == ScoreLocation.L1) {
            flags.check(RobotFlag.SCORE_GROUND);
        } else {
            flags.check(RobotFlag.SCORE_ARM);
        }
    }

    public void climbRequest() {
        flags.check(RobotFlag.CLIMB);
    }

    public void scoreLevelRequest() {
        if (operatorOptions.scoreLocation == ScoreLocation.L1) {
            flags.check(RobotFlag.PREPARE_SCORE_GROUND);
        } else {
            flags.check(RobotFlag.PREPARE_SCORE_ARM);
        }
    }

    public void invertedHandoffRequest() {
        flags.check(RobotFlag.INVERTED_HANDOFF);
    }

    public void intakeAlgaeRequest() {
        flags.check(RobotFlag.INTAKE_ALGAE);
    }

    public void algaeScoreLevelRequest() {
        flags.check(RobotFlag.INTAKE_ALGAE);
    }

    public void handoffRequest() {
        flags.check(RobotFlag.HANDOFF);
    }

    public void resetToIdleRequest() {
        flags.check(RobotFlag.RESET_TO_IDLE);
    }

    public void groundIdleRequest() {
        flags.check(RobotFlag.GROUND_IDLE);
    }

    public void setL1() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L1;
        DogLog.log("Robot/ScoreLocation", "L1");
    }

    public void setL2() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L2;
        DogLog.log("Robot/ScoreLocation", "L2");
    }

    public void setL3() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L3;
        DogLog.log("Robot/ScoreLocation", "L3");
    }

    public void setL4() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L4;
        DogLog.log("Robot/ScoreLocation", "L4");
    }

    public void setProcessor() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.PROCESSOR;
        DogLog.log("Robot/ScoreLocation", "PROCESSOR");
    }

    public void setBarge() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.BARGE;
        DogLog.log("Robot/ScoreLocation", "BARGE");

    }

    public void toggleCoralMode() {
        if (operatorOptions.coralMode == CoralMode.CORAL_MODE) {
            operatorOptions.coralMode = CoralMode.NORMAL_MODE;
            DogLog.log("Robot/CoralMode", "NORMAL");
        } else {
            operatorOptions.coralMode = CoralMode.CORAL_MODE;
            DogLog.log("Robot/CoralMode", "CORAL");
        }
    }

    public void setHighReefAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.HIGH_REEF;
        DogLog.log("Robot/AlgaeLocation", "HIGH_REEF");
    }

    public void setLowReefAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.LOW_REEF;
        DogLog.log("Robot/AlgaeLocation", "LOW_REEF");
    }

    public void setGroundAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE;
        DogLog.log("Robot/AlgaeLocation", "GROUND");
    }

    private static RequestManager instance;

    public static RequestManager getInstance() {
        if (instance == null)
            instance = new RequestManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
