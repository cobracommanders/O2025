package frc.robot.stateMachine;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.armManager.ArmManager;
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

    private final FlagManager<RobotFlag> flags = new FlagManager<>("RequestManager", RobotFlag.class);
    private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

    public static final double HANDOFF_TIME = 0.5;
    public static final double INVERTED_HANDOFF_TIME = 0.5;

    private RequestManager() {
        super(RequestManagerState.INDEPENDENT);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (getState() == RequestManagerState.INDEPENDENT) {
            sendIndependentManagerRequests(); // this is kind of tragic
        }
    }

    @Override
    protected void collectInputs() {
        // Log all flags before they are modified in later methods
        flags.log();
    }

    @Override
    protected RequestManagerState getNextState(RequestManagerState currentState) {
        RequestManagerState nextState = currentState;

        for (RobotFlag flag : flags.getChecked()) {
            if (armManager.atPosition()) {
                switch (flag) {
                    case CLIMB -> {
                        flags.remove(flag);
                        nextState = RequestManagerState.CLIMB;
                    }
                    case HANDOFF -> {
                        flags.remove(flag);
                        nextState = RequestManagerState.PREPARE_HANDOFF_GROUND;
                    }
                    case INVERTED_HANDOFF -> {
                        flags.remove(flag);
                        nextState = RequestManagerState.PREPARE_INVERTED_HANDOFF;
                    }
                    case RESET_TO_IDLE -> {
                        flags.remove(flag);
                        nextState = RequestManagerState.PREPARE_IDLE;
                    }
                    default -> {
                    }
                }
            }
        }

        switch (currentState) {
            case PREPARE_HANDOFF_GROUND -> {
                if (groundManager.getState() == GroundManagerStates.WAIT_HANDOFF) {
                    nextState = RequestManagerState.PREPARE_HANDOFF_ARM;
                }
            }
            case PREPARE_HANDOFF_ARM -> {
                if (armManager.isReadyToExecuteHandoff()) {
                    nextState = RequestManagerState.HANDOFF;
                }
            }
            case HANDOFF -> {
                if (timeout(HANDOFF_TIME)) {
                    nextState = RequestManagerState.INDEPENDENT;
                }
            }

            case PREPARE_INVERTED_HANDOFF -> {
                if (armManager.isReadyToExecuteInvertedHandoff() && groundManager.getState() == GroundManagerStates.WAIT_INVERTED_HANDOFF) {
                    nextState = RequestManagerState.INVERTED_HANDOFF;
                }
            }
            case INVERTED_HANDOFF -> {
                if (coralDetector.hasCoral() || timeout(0.5)) {
                    nextState = RequestManagerState.INDEPENDENT;
                }
            }
            case PREPARE_IDLE -> {
                if (armManager.isIdleState() && groundManager.getState() == GroundManagerStates.IDLE) {
                    nextState = RequestManagerState.INDEPENDENT;
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
                groundManager.setState(GroundManagerStates.PREPARE_IDLE);
                armManager.requestIdleClearGamePiece();
            }

            case PREPARE_HANDOFF_GROUND -> {
                groundManager.setState(GroundManagerStates.PREPARE_HANDOFF);
            }
            case PREPARE_HANDOFF_ARM -> {
                armManager.requestHandoff();
            }

            case HANDOFF -> {
                groundManager.setState(GroundManagerStates.HANDOFF);
                armManager.requestHandoffExecution();
            }

            case PREPARE_INVERTED_HANDOFF -> {
                groundManager.setState(GroundManagerStates.PREPARE_INVERTED_HANDOFF);
                armManager.requestInvertedHandoff();
            }
            case INVERTED_HANDOFF -> {
                groundManager.setState(GroundManagerStates.PREPARE_INVERTED_HANDOFF);
                armManager.requestInvertedHandoffExecution();
            }

            case CLIMB -> {
                climber.setState(ClimberStates.DEPLOYING);
                groundManager.setState(GroundManagerStates.CLIMB);
                armManager.requestClimb();
            }
            case INDEPENDENT -> {
                groundManager.setState(GroundManagerStates.IDLE);
            }
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
                        case GROUND_ALGAE -> armManager.requestGroundAlgaeIntake();
                        case HIGH_REEF, LOW_REEF -> {
                            boolean top;
                            if (FeatureFlags.AUTO_ALGAE_INTAKE_HEIGHT.getAsBoolean()) {
                                top = AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3;
                            } else {
                                top = operatorOptions.algaeIntakeLevel == OperatorOptions.AlgaeIntakeLevel.HIGH_REEF;
                            }
                            armManager.requestReefAlgaeIntake(AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose()), top);
                        }
                    }
                }
                case INTAKE_CORAL_LOLLIPOP -> {
                    flags.remove(flag);
                    if (DriverStation.isAutonomous()) {
                        armManager.requestLollipopIntake();
                    }
                }
                case INTAKE_CORAL -> {
                    flags.remove(flag);
                    groundManager.setState(GroundManagerStates.PREPARE_INTAKE);
                }
                case PREPARE_SCORE_ARM -> {
                    flags.remove(flag);

                    var coralRobotSide = AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
                    var algaeRobotSide = AutoAlign.getNetScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
                    switch (operatorOptions.scoreLocation) {
                        case L2 -> armManager.requestCoralPrepare(coralRobotSide, FieldConstants.PipeScoringLevel.L2);
                        case L3 -> armManager.requestCoralPrepare(coralRobotSide, FieldConstants.PipeScoringLevel.L3);
                        case L4 -> armManager.requestCoralPrepare(coralRobotSide, FieldConstants.PipeScoringLevel.L4);
                        case BARGE -> armManager.requestAlgaeNetScore(algaeRobotSide);
                        case PROCESSOR -> armManager.requestProcessorScore();
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
                        case L2, L3, L4 -> armManager.requestCoralScoreExecution();
                        case BARGE -> armManager.requestAlgaeNetScoreExecution();
                        case PROCESSOR -> armManager.requestProcessorScoreExecution();
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

    private static RequestManager instance;

    public static RequestManager getInstance() {
        if (instance == null)
            instance = new RequestManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
