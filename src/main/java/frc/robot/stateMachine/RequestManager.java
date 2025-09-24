package frc.robot.stateMachine;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.stateMachine.OperatorOptions.CoralMode;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

public class RequestManager extends StateMachine<RequestManagerState> {
    private final ArmManager armManager = Robot.armManager;
    private final GroundManager groundManager = GroundManager.getInstance();
    private final Climber climber = Climber.getInstance();
    private final CoralDetector coralDetector = CoralDetector.getInstance();
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    private final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);
    private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

    private RequestManager() {
        super(RequestManagerState.INDEPENDENT);
    }

    @Override
    protected void collectInputs() {
        // Log all flags before they are modified in later methods
        flags.log();

        DogLog.log("coral mode", operatorOptions.coralMode);
    }

    @Override
    protected RequestManagerState getNextState(RequestManagerState currentState) {
        RequestManagerState nextState = currentState;

        // TODO I think this maybe should go in the INDEPENDENT/CLIMB state switch block because it's only not overridden when that is active. Unless it's here to make sure those flags are cleared every loop.
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case CLIMB -> {
                    if (armManager.isReady()) {
                        nextState = RequestManagerState.CLIMB;
                        flags.remove(flag);
                    }
                }
                case HANDOFF -> {
                    if (armManager.isReady()) {
                        nextState = RequestManagerState.PREPARE_HANDOFF;
                        flags.remove(flag);
                    }
                }
                case INVERTED_HANDOFF -> {
                    if (armManager.isReady()) {
                        nextState = RequestManagerState.PREPARE_INVERTED_HANDOFF;
                        flags.remove(flag);
                    }
                }
                case RESET_TO_IDLE -> {
                    if (armManager.isReady()) {
                        nextState = RequestManagerState.PREPARE_IDLE;
                        flags.remove(flag);
                    }
                }
                default -> {
                }
            }
        }


        switch (currentState) {
            case PREPARE_HANDOFF -> {
                // TODO why not ArmManagerStates.WAIT_HANDOFF_CORAL_MODE as well?
                if ((armManager.getState() == ArmManagerState.WAIT_HANDOFF_LEFT ||
                        armManager.getState() == ArmManagerState.WAIT_HANDOFF_MIDDLE ||
                        armManager.getState() == ArmManagerState.WAIT_HANDOFF_RIGHT) &&
                        groundManager.getState() == GroundManagerStates.WAIT_HANDOFF) {
                    nextState = RequestManagerState.HANDOFF;
                }
            }
            case HANDOFF -> {
                // Waits 0.1 seconds in normal mode and 0.2 seconds in coral mode
                // TODO is the handoff the same?
                if ((operatorOptions.coralMode == CoralMode.NORMAL_MODE && timeout(0.1)) || timeout(0.2)) {
                    nextState = RequestManagerState.INDEPENDENT;
                }
            }
            case PREPARE_INVERTED_HANDOFF -> {
                if ((armManager.getState() == ArmManagerState.WAIT_INVERTED_HANDOFF) &&
                        groundManager.getState() == GroundManagerStates.WAIT_INVERTED_HANDOFF) {
                    nextState = RequestManagerState.INVERTED_HANDOFF;
                }
            }
            case INVERTED_HANDOFF -> {
                if (coralDetector.hasCoral() || timeout(0.5)) {
                    nextState = RequestManagerState.INDEPENDENT;
                }
            }
            case PREPARE_IDLE -> {
                // Wait for either idle condition based on the current mode
                if (operatorOptions.coralMode == CoralMode.NORMAL_MODE) {
                    if (armManager.getState() == ArmManagerState.IDLE && groundManager.getState() == GroundManagerStates.IDLE) {
                        nextState = RequestManagerState.INDEPENDENT;
                    }
                } else if (operatorOptions.coralMode == CoralMode.CORAL_MODE) {
                    if (armManager.getState() == ArmManagerState.WAIT_HANDOFF_MIDDLE && groundManager.getState() == GroundManagerStates.IDLE) {
                        nextState = RequestManagerState.INDEPENDENT;
                    }
                }
            }
            case CLIMB, INDEPENDENT -> {
            }
        }

        return nextState;
    }

    @Override
    protected void afterTransition(RequestManagerState newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                if (operatorOptions.coralMode == CoralMode.CORAL_MODE) {
                    groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                    armManager.setState(ArmManagerState.PREPARE_HANDOFF_MIDDLE);
                } else if (operatorOptions.coralMode == CoralMode.NORMAL_MODE) {
                    groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                    armManager.setState(ArmManagerState.PREPARE_IDLE);
                }
            }

            case PREPARE_HANDOFF -> {
                // need to figure out which handoff we are doing...
                // TODO figure out what this if statement is doing and why it isn't in the inverted case
                if (armManager.getState() != ArmManagerState.WAIT_HANDOFF_CORAL_MODE) {
                    armManager.setState(ArmManagerState.PREPARE_HANDOFF_CORAL_MODE);
                }
                groundManager.setState(GroundManagerStates.PREPARE_HANDOFF);
            }
            case HANDOFF -> groundManager.setState(GroundManagerStates.HANDOFF);

            case PREPARE_INVERTED_HANDOFF -> {
                armManager.setState(ArmManagerState.PREPARE_INVERTED_HANDOFF);
                groundManager.setState(GroundManagerStates.PREPARE_INVERTED_HANDOFF);
            }
            case INVERTED_HANDOFF -> armManager.setState(ArmManagerState.INVERTED_HANDOFF);

            case CLIMB -> {
                climber.setState(ClimberStates.DEPLOYING);
                armManager.setState(ArmManagerState.CLIMB);
                groundManager.setState(GroundManagerStates.CLIMB);
            }
            case INDEPENDENT -> sendIndependentManagerRequests();
        }
    }

    /**
     * Reads the set flags and sends any necessary requests to control the arm and ground subsystems in parallel.
     */
    private void sendIndependentManagerRequests() {
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case INTAKE_ALGAE -> {
                    flags.remove(flag);

                    switch (operatorOptions.algaeIntakeLevel) {
                        case GROUND_ALGAE -> armManager.setState(ArmManagerState.PREPARE_INTAKE_GROUND_ALGAE);
                        // TODO Could maybe be one REEF state unless it's here for ease of switching back
                        case HIGH_REEF -> {
                            if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L2) {
                                armManager.setState(ArmManagerState.PREPARE_INTAKE_LOW_REEF_ALGAE);
                            }
                            armManager.setState(ArmManagerState.PREPARE_INTAKE_HIGH_REEF_ALGAE);
                        }
                        case LOW_REEF -> {
                            if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3) {
                                armManager.setState(ArmManagerState.PREPARE_INTAKE_HIGH_REEF_ALGAE);
                            }
                            armManager.setState(ArmManagerState.PREPARE_INTAKE_LOW_REEF_ALGAE);
                        }
                    }
                }
                case INTAKE_CORAL_LOLLIPOP -> {
                    flags.remove(flag);
                    if (DriverStation.isAutonomous()) {
                        armManager.setState(ArmManagerState.PREPARE_INTAKE_LOLLIPOP);
                    }
                }
                case INTAKE_CORAL -> {
                    flags.remove(flag);
                    groundManager.setState(GroundManagerStates.PREPARE_INTAKE);
                }
                case PREPARE_SCORE_ARM -> {
                    flags.remove(flag);

                    switch (operatorOptions.scoreLocation) {
                        case L2 -> armManager.setState(ArmManagerState.PREPARE_SCORE_L2);
                        case L3 -> armManager.setState(ArmManagerState.PREPARE_SCORE_L3);
                        case L4 -> armManager.setState(ArmManagerState.PREPARE_SCORE_L4);
                        case BARGE -> armManager.setState(ArmManagerState.PREPARE_SCORE_ALGAE_NET);
                        case PROCESSOR -> armManager.setState(ArmManagerState.PREPARE_SCORE_ALGAE_PROCESSOR);
                        default -> {
                        }
                    }
                }
                case PREPARE_SCORE_GROUND -> {
                    flags.remove(flag);

                    if (operatorOptions.scoreLocation == ScoreLocation.L1) {
                        groundManager.setState(GroundManagerStates.PREPARE_SCORE_L1);
                    }
                }
                case SCORE_ARM -> {
                    flags.remove(flag);

                    switch (operatorOptions.scoreLocation) {
                        case L2 -> armManager.setState(ArmManagerState.SCORE_L2);
                        case L3 -> armManager.setState(ArmManagerState.SCORE_L3);
                        case L4 -> armManager.setState(ArmManagerState.SCORE_L4);
                        case BARGE -> armManager.setState(ArmManagerState.SCORE_ALGAE_NET);
                        case PROCESSOR -> armManager.setState(ArmManagerState.SCORE_ALGAE_PROCESSOR);
                        default -> {
                        }
                    }
                }
                case SCORE_GROUND -> {
                    flags.remove(flag);

                    if (operatorOptions.scoreLocation == ScoreLocation.L1) {
                        groundManager.setState(GroundManagerStates.SCORE_L1);
                    }
                }
                case GROUND_IDLE -> {
                    flags.remove(flag);
                    groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                }
                default -> {
                }
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

    public void handoffRequest() {
        flags.check(RobotFlag.HANDOFF);
    }

    public void invertedHandoffRequest() {
        flags.check(RobotFlag.INVERTED_HANDOFF);
    }

    public void intakeAlgaeRequest() {
        flags.check(RobotFlag.INTAKE_ALGAE);
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
