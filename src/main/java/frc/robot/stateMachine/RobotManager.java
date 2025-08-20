package frc.robot.stateMachine;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorStates;

public class RobotManager extends StateMachine<RobotState> {

    public final ArmManager armManager;
    public final GroundManager groundManager;
    public final Climber climber;
    public final CoralDetector coralDetector;
    public OperatorOptions operatorOptions = OperatorOptions.getInstance();

    public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

    public RobotManager() {
        super(RobotState.IDLE);

        this.armManager = ArmManager.getInstance();
        this.groundManager = GroundManager.getInstance();
        this.climber = Climber.getInstance();
        this.coralDetector = CoralDetector.getInstance();
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
                    if (getState() == RobotState.IDLE) {
                        nextState = RobotState.INTAKING_CORAL;
                    }
                    break;
                case CLIMB:
                    if (getState() == RobotState.IDLE) {
                        nextState = RobotState.CLIMB;
                    }
                    break;
                case HANDOFF:
                    if (getState() == RobotState.IDLE) {
                        nextState = RobotState.PREPARE_HANDOFF;
                    }
                    break;
                case SCORE:
                    switch (nextState) {
                        case WAIT_L1:
                            nextState = RobotState.SCORE_L1;
                            break;
                        case WAIT_L4:
                            nextState = RobotState.SCORE_L4;
                            break;
                        case WAIT_BARGE:
                            nextState = RobotState.SCORE_BARGE;
                            break;
                        case WAIT_PROCESSOR:
                            nextState = RobotState.SCORE_PROCESSOR;
                            break;
                        default:
                            break;
                    }
                    break;
                case SCORE_LEVEL:
                    switch (operatorOptions.scoreLocation) {
                        case L1:
                            nextState = RobotState.WAIT_L1;
                            break;
                        case L4:
                            if (coralDetector.hasCoral()) {
                                nextState = RobotState.PREPARE_HANDOFF;
                            } else {
                                nextState = RobotState.WAIT_L4;
                            }
                            break;
                        case BARGE:
                            if (getState() == RobotState.IDLE)
                                nextState = RobotState.WAIT_BARGE;
                            break;
                        case PROCESSOR:
                            if (getState() == RobotState.IDLE)
                                nextState = RobotState.WAIT_PROCESSOR;
                            break;
                        default:
                            break;
                    }
                    break;
                case INTAKE_ALGAE:
                    switch (operatorOptions.algaeIntakeLevel) {
                        case HIGH_REEF:
                            if (armManager.hand.hasAlgae() || armManager.hand.hasCoral()) {
                            } else {
                                nextState = RobotState.HIGH_REEF_ALGAE_INTAKE;
                            }
                            break;
                        case LOW_REEF:
                            if (armManager.hand.hasAlgae() || armManager.hand.hasCoral()) {
                            } else {
                                nextState = RobotState.LOW_REEF_ALGAE_INTAKE;
                            }
                            break;
                        case GROUND_ALGAE:
                            if (armManager.hand.hasAlgae() || armManager.hand.hasCoral()) {
                            } else {
                                nextState = RobotState.GROUND_ALGAE_INTAKE;
                            }
                            nextState = RobotState.GROUND_ALGAE_INTAKE;
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

            case WAIT_L1, WAIT_L4, WAIT_BARGE, WAIT_PROCESSOR, CLIMB, IDLE:
                // These states do not transition automatically
                break;

            case INTAKING_CORAL:
                if (groundManager.getState() == GroundManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case PREPARE_HANDOFF:
                if ((armManager.getState() == ArmManagerStates.WAIT_HANDOFF_LEFT ||
                        armManager.getState() == ArmManagerStates.WAIT_HANDOFF_MIDDLE ||
                        armManager.getState() == ArmManagerStates.WAIT_HANDOFF_RIGHT) &&
                        groundManager.getState() == GroundManagerStates.WAIT_HANDOFF) {
                    nextState = RobotState.HANDOFF;
                }
                break;
            case HANDOFF:
                if (timeout(0.3)) {
                    nextState = RobotState.WAIT_L4;
                    // will check Operator Options when we are scoring on more Levels
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
            case SCORE_BARGE:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
            case SCORE_PROCESSOR:
                if (armManager.getState() == ArmManagerStates.IDLE) {
                    nextState = RobotState.IDLE;
                }
                break;
        }

        flags.clear();
        return nextState;

    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            case IDLE -> {
            }
            case WAIT_BARGE ->{
                armManager.setState(ArmManagerStates.PREPARE_SCORE_ALGAE_NET);
                
            }
            case WAIT_PROCESSOR -> {
                armManager.setState(ArmManagerStates.PREPARE_SCORE_ALGAE_PROCESSOR);
            }
            case HANDOFF -> {
                groundManager.setStateFromRequest(GroundManagerStates.HANDOFF);
            }
            case SCORE_L1 -> {
                groundManager.setState(GroundManagerStates.SCORE_L1);
            }
            case SCORE_L4 -> {
                armManager.setState(ArmManagerStates.SCORE_L4);
            }
            case SCORE_PROCESSOR -> {
                armManager.setState(ArmManagerStates.SCORE_ALGAE_PROCESSOR);
            }
            case SCORE_BARGE -> {
                armManager.setState(ArmManagerStates.SCORE_ALGAE_NET);
            }
            case PREPARE_HANDOFF -> {
                // need to figure out which handoff we are doing...
                if(coralDetector.getState() == CoralDetectorStates.LEFT){
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_LEFT);
                }else if(coralDetector.getState() == CoralDetectorStates.RIGHT){
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_RIGHT);
                }else if(coralDetector.getState() == CoralDetectorStates.MIDDLE){
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_MIDDLE);
                }else{
                    armManager.setState(ArmManagerStates.PREPARE_HANDOFF_MIDDLE);
                    //if there is no coral, do nothing
                }
                groundManager.setState(GroundManagerStates.PREPARE_HANDOFF);
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
                groundManager.setState(GroundManagerStates.PREPARE_INTAKE);
            }
            case WAIT_L1 -> {
                groundManager.setState(GroundManagerStates.PREPARE_SCORE_L1);
            }
            case WAIT_L4 -> {
                armManager.setState(ArmManagerStates.PREPARE_SCORE_L4);
                groundManager.setState(GroundManagerStates.PREPARE_IDLE);
            }
            case CLIMB -> {
                climber.setState(ClimberStates.DEPLOYING);
                //TODO: add intake and armManager climb states
            }
        }
    }
    

    @Override
    public void periodic() {
        super.periodic();
    }

    public void coralIntakeRequest() {
        flags.check(RobotFlag.INTAKE_CORAL);
    }

    public void scoreRequest() {
        flags.check(RobotFlag.SCORE);
    }

    public void climbRequest() {
        flags.check(RobotFlag.CLIMB);
    }

    public void scoreLevelRequest() {
        flags.check(RobotFlag.SCORE_LEVEL);
    }

    public void idleRequest() {
        flags.check(RobotFlag.IDLE);
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

    public void setL1() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L1;
    }

    public void setL2() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L2;
    }

    public void setL3() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L3;
    }

    public void setL4() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.L4;
    }

    public void setProcessor() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.PROCESSOR;
    }

    public void setBarge() {
        operatorOptions.scoreLocation = OperatorOptions.ScoreLocation.BARGE;
    }

    public void setHighReefAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.HIGH_REEF;
    }

    public void setLowReefAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.LOW_REEF;
    }

    public void setGroundAlgae() {
        operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE;
    }

    private static RobotManager instance;

    public static RobotManager getInstance() {
        if (instance == null)
            instance = new RobotManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
