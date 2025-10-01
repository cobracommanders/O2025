package frc.robot.subsystems.armManager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.tagAlign.TagAlign;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.OperatorOptions.CoralMode;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmPositions;
import frc.robot.subsystems.armManager.arm.ArmStates;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.armManager.armScheduler.ArmSchedulerStates;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorPositions;
import frc.robot.subsystems.armManager.elevator.ElevatorStates;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandSpeeds;
import frc.robot.subsystems.armManager.hand.HandStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorStates;

public class ArmManager extends StateMachine<ArmManagerStates> {
    public static final double scoringTime = 0.75;
    private final String name = getName();
    public final Hand hand;
    public final Elevator elevator;
    public final Arm arm;
    public final CoralDetector coralDetector;

    private boolean synced = false;

    public final ArmScheduler armScheduler;

    public ArmManager() {
        super(ArmManagerStates.PREPARE_IDLE);

        this.hand = Hand.getInstance();
        this.elevator = Elevator.getInstance();
        this.arm = Arm.getInstance();
        this.armScheduler = ArmScheduler.getInstance();
        this.coralDetector = CoralDetector.getInstance();
    }

    public boolean isReadyToMove(){
        if (DriverStation.isAutonomous()) return false;
       return AutoAlign.getInstance().usedScoringPose.getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 0.2;
    }

    @Override
    public void periodic(){
        super.periodic();
    }

    protected ArmManagerStates getNextState(ArmManagerStates currentState) {
        ArmManagerStates nextState = currentState;

        switch (currentState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (atGoal()) {
                    nextState = ArmManagerStates.INTAKE_LOLLIPOP;
                }
            }
            case INTAKE_LOLLIPOP -> {
                
            }
            case PREPARE_IDLE -> {
                if (armScheduler.isReady() ) {
                    nextState = ArmManagerStates.IDLE;
                }
            }
            case PREPARE_SCORE_L4 -> {
                // if (DriverStation.isAutonomous() && atGoal()) {
                //     nextState = ArmManagerStates.WAIT_L4;
                // }
                // else if (DriverStation.isTeleop() && armScheduler.isReady()){
                //     nextState = ArmManagerStates.WAIT_L4;
                // }
                if(armScheduler.isReady()){
                    nextState = ArmManagerStates.WAIT_L4;
                }
            }

            case PREPARE_SCORE_L2 -> {
                // if (DriverStation.isAutonomous() && atGoal()) {
                //     nextState = ArmManagerStates.WAIT_L4;
                // }
                // else if (DriverStation.isTeleop() && armScheduler.isReady()){
                //     nextState = ArmManagerStates.WAIT_L4;
                // }
                if(armScheduler.isReady()){
                    nextState = ArmManagerStates.WAIT_L2;
                }
            }

            case SCORE_L4 -> {
                if(isReadyToMove()) {
                    if(RequestManager.getInstance().operatorOptions.coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                        nextState = ArmManagerStates.PREPARE_IDLE;
                    }else{
                        nextState = ArmManagerStates.PREPARE_HANDOFF_CORAL_MODE;
                    }
                }
            }
            case SCORE_L3 -> {
                if(isReadyToMove()){
                    if(RequestManager.getInstance().operatorOptions.coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                        nextState = ArmManagerStates.PREPARE_IDLE;
                    }else{
                        nextState = ArmManagerStates.PREPARE_HANDOFF_CORAL_MODE;
                    }
                }
            }
            case SCORE_L2 -> {
                if(isReadyToMove()){
                    if(RequestManager.getInstance().operatorOptions.coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                        nextState = ArmManagerStates.PREPARE_IDLE;
                    }else{
                        nextState = ArmManagerStates.PREPARE_HANDOFF_CORAL_MODE;
                    }
                }
            }

            case PREPARE_HANDOFF_LEFT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                // if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_LEFT;
                }
            }

            case PREPARE_HANDOFF_RIGHT -> {
                if (arm.atGoal() && elevator.atGoal()) {
                // if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_RIGHT;
                }
            }

            case PREPARE_HANDOFF_MIDDLE -> {
                if (arm.atGoal() && elevator.atGoal()) {
                // if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_MIDDLE;
                }
            }

            case PREPARE_HANDOFF_CORAL_MODE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_HANDOFF_CORAL_MODE;
                }
            }

            case WAIT_HANDOFF_CORAL_MODE -> {
                if(GroundManager.getInstance().getState() == GroundManagerStates.WAIT_HANDOFF && coralDetector.hasCoral()) {
                    if (coralDetector.getState() == CoralDetectorStates.LEFT) {
                        nextState = ArmManagerStates.PREPARE_HANDOFF_LEFT;
                    } else if (coralDetector.getState() == CoralDetectorStates.RIGHT) {
                        nextState = ArmManagerStates.PREPARE_HANDOFF_RIGHT;
                    } else if (coralDetector.getState() == CoralDetectorStates.MIDDLE) {
                        nextState = ArmManagerStates.PREPARE_HANDOFF_MIDDLE;
                    } else {
                        nextState = ArmManagerStates.PREPARE_HANDOFF_MIDDLE;
                    // if there is no coral, do nothing
                    }
                }
            }

            case PREPARE_INTAKE_GROUND_ALGAE -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_GROUND_ALGAE;
                }
            }
            case INTAKE_GROUND_ALGAE -> {
                if (hand.hasAlgae()) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3){
                    nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                }
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_HIGH_REEF_ALGAE;
                }
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L2){
                     nextState = ArmManagerStates.PREPARE_INTAKE_LOW_REEF_ALGAE;
                 }
                else {
                    nextState = ArmManagerStates.ALGAE_LEAVE_REEF;
                }
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3){
                    nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                }
                else if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.INTAKE_LOW_REEF_ALGAE;
                }
            }
            case INTAKE_LOW_REEF_ALGAE -> {
                if (AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3){
                     nextState = ArmManagerStates.PREPARE_INTAKE_HIGH_REEF_ALGAE;
                 }
                 else {
                    nextState = ArmManagerStates.ALGAE_LEAVE_REEF;
                }
            }
            case ALGAE_LEAVE_REEF -> {
                if(AutoAlign.getInstance().getAlgaeDistance().getTranslation().getDistance(LocalizationSubsystem.getInstance().getPose2d().getTranslation()) >= 1.25) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_SCORE_ALGAE_NET -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_NET;
                }
            }
            case WAIT_SCORE_ALGAE_NET -> {
            }
            case SCORE_ALGAE_NET -> {
                if (timeout(1)) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_SCORE_ALGAE_PROCESSOR;
                }
            }
            case WAIT_SCORE_ALGAE_PROCESSOR -> {
            }
            case SCORE_ALGAE_PROCESSOR -> {
                if (timeout(1)) {
                    nextState = ArmManagerStates.PREPARE_IDLE;
                }
            }
            case PREPARE_INVERTED_HANDOFF -> {
                if (armScheduler.isReady()) {
                    nextState = ArmManagerStates.WAIT_INVERTED_HANDOFF;
                }
            }
            case WAIT_INVERTED_HANDOFF -> {
            }

        }

        return nextState;
    }

    public void setState(ArmManagerStates state) {
        if (armScheduler.isReady()) {
            setStateFromRequest(state);
        }else{
            //do nothing
        }
    }

    public boolean isReady() {
        return armScheduler.isReady();
    }

    @Override
    protected void afterTransition(ArmManagerStates newState) {
        synced = false;
        switch (newState) {
            case PREPARE_INTAKE_LOLLIPOP -> {
                if (DriverStation.isAutonomous()) {
                    arm.setState(ArmStates.LOLLIPOP);
                    hand.setState(HandStates.LOLLIPOP);
                    elevator.setState(ElevatorStates.LOLLIPOP);
                } else {
                    armScheduler.scheduleStates(ArmStates.LOLLIPOP, HandStates.LOLLIPOP, ElevatorStates.LOLLIPOP);
                }
            }
            case INTAKE_LOLLIPOP -> {

            }
            case PREPARE_IDLE -> {
                armScheduler.scheduleStates(ArmStates.IDLE, HandStates.IDLE, ElevatorStates.IDLE);
            }
            case IDLE -> {
                if (!synced && timeout(1)) {
                    arm.syncEncoder();
                    synced = true;
                }
            }
            case PREPARE_INTAKE_GROUND_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_GROUND_ALGAE, HandStates.INTAKE_GROUND_ALGAE,
                        ElevatorStates.GROUND_ALGAE);
            }
            case INTAKE_GROUND_ALGAE -> {
            }
            case PREPARE_INTAKE_HIGH_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_HIGH_REEF_ALGAE, HandStates.INTAKE_HIGH_REEF_ALGAE,
                        ElevatorStates.HIGH_REEF_ALGAE);
            }
            case INTAKE_HIGH_REEF_ALGAE -> {
            }
            case PREPARE_INTAKE_LOW_REEF_ALGAE -> {
                armScheduler.scheduleStates(ArmStates.INTAKE_LOW_REEF_ALGAE, HandStates.INTAKE_LOW_REEF_ALGAE,
                        ElevatorStates.LOW_REEF_ALGAE);
            }
            case INTAKE_LOW_REEF_ALGAE -> {
            }
            case PREPARE_SCORE_ALGAE_NET -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_NET, HandStates.IDLE, ElevatorStates.ALGAE_NET);
            }
            case WAIT_SCORE_ALGAE_NET -> {
            }
            case SCORE_ALGAE_NET -> {
                hand.setState(HandStates.SCORE_ALGAE_NET);
            }
            case PREPARE_SCORE_ALGAE_PROCESSOR -> {
                armScheduler.scheduleStates(ArmStates.ALGAE_PROCESSOR, HandStates.IDLE, ElevatorStates.ALGAE_PROCESSOR);
            }
            case WAIT_SCORE_ALGAE_PROCESSOR -> {
            }
            case SCORE_ALGAE_PROCESSOR -> {
                hand.setState(HandStates.SCORE_ALGAE_PROCESSOR);
            }
            case PREPARE_SCORE_L4 -> {
                // arm.setState(ArmStates.L4);
                // hand.setState(HandStates.CORAL_IDLE);
                // elevator.setState(ElevatorStates.L4);
                // if (DriverStation.isAutonomous()) {
                //     arm.setState(ArmStates.L4);
                //     hand.setState(HandStates.CORAL_IDLE);
                //     elevator.setState(ElevatorStates.L4);
                // } else {
                //     armScheduler.scheduleStates(ArmStates.L4, HandStates.CORAL_IDLE, ElevatorStates.L4);
                // }
                armScheduler.scheduleStates(ArmStates.L4, HandStates.CORAL_IDLE, ElevatorStates.L4);
            }
            case WAIT_L4 -> {
                // armScheduler.scheduleStates(ArmStates.L4, HandStates.CORAL_IDLE, ElevatorStates.L4);
            }
            case SCORE_L4 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L4);
                arm.setState(ArmStates.SCORE_L4);
            }
            case PREPARE_SCORE_L3 -> {
                armScheduler.scheduleStates(ArmStates.L3, HandStates.CORAL_IDLE, ElevatorStates.L3);
            }
            case WAIT_L3 -> {
                armScheduler.scheduleStates(ArmStates.L3, HandStates.CORAL_IDLE, ElevatorStates.L3);
            }
            case SCORE_L3 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L3);
                arm.setState(ArmStates.SCORE_L3);
            }
            case PREPARE_SCORE_L2 -> {
                armScheduler.scheduleStates(ArmStates.L2, HandStates.CORAL_IDLE, ElevatorStates.L2);
            }
            case WAIT_L2 -> {
                //armScheduler.scheduleStates(ArmStates.L2, HandStates.CORAL_IDLE, ElevatorStates.L2);
            }
            case SCORE_L2 -> {
                hand.setState(HandStates.SCORE_CORAL);
                elevator.setState(ElevatorStates.SCORE_L2);
                arm.setState(ArmStates.SCORE_L2);
            }
            case PREPARE_HANDOFF_RIGHT -> {
                arm.setState(ArmStates.HANDOFF_RIGHT);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
                //if normal mode, we need to use arm scheduler, if not, we can ignore arm scheduler
                // if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.CORAL_MODE){
                //     arm.setState(ArmStates.HANDOFF_RIGHT);
                //     elevator.setState(ElevatorStates.HANDOFF);
                //     hand.setState(HandStates.HANDOFF);
                // }
                // else if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                //     armScheduler.scheduleStates(ArmStates.HANDOFF_RIGHT, HandStates.HANDOFF, ElevatorStates.HANDOFF);
                // }
            }
            case WAIT_HANDOFF_RIGHT -> {
            }
            case PREPARE_HANDOFF_MIDDLE -> {
                arm.setState(ArmStates.HANDOFF_MIDDLE);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
                //if normal mode, we need to use arm scheduler, if not, we can ignore arm scheduler
                // if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.CORAL_MODE){
                //     arm.setState(ArmStates.HANDOFF_MIDDLE);
                //     elevator.setState(ElevatorStates.HANDOFF);
                //     hand.setState(HandStates.HANDOFF);
                // }
                // else if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                //     armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.HANDOFF, ElevatorStates.HANDOFF);
                // }
            }
            case WAIT_HANDOFF_MIDDLE -> {
            }
            case PREPARE_HANDOFF_LEFT -> {
                arm.setState(ArmStates.HANDOFF_LEFT);
                elevator.setState(ElevatorStates.HANDOFF);
                hand.setState(HandStates.HANDOFF);
                //if normal mode, we need to use arm scheduler, if not, we can ignore arm scheduler
                // if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.CORAL_MODE){
                //     arm.setState(ArmStates.HANDOFF_LEFT);
                //     elevator.setState(ElevatorStates.HANDOFF);
                //     hand.setState(HandStates.HANDOFF);
                // }
                // else if (OperatorOptions.getInstance().coralMode == OperatorOptions.CoralMode.NORMAL_MODE){
                //     armScheduler.scheduleStates(ArmStates.HANDOFF_LEFT, HandStates.HANDOFF, ElevatorStates.HANDOFF);
                // }
            }
            case WAIT_HANDOFF_LEFT -> {
            }
            case PREPARE_HANDOFF_CORAL_MODE -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.HANDOFF, ElevatorStates.HANDOFF_CORAL_MODE);
            }
            case WAIT_HANDOFF_CORAL_MODE -> {
                //armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.HANDOFF, ElevatorStates.HANDOFF_CORAL_MODE);
            }
            case CLIMB -> {
                armScheduler.scheduleStates(ArmStates.CLIMB, HandStates.IDLE, ElevatorStates.IDLE);
            }
            case PREPARE_INVERTED_HANDOFF -> {
                armScheduler.scheduleStates(ArmStates.HANDOFF_MIDDLE, HandStates.IDLE, ElevatorStates.HANDOFF);
            }
            case WAIT_INVERTED_HANDOFF -> {
            }

        }
    }

    public Command waitForGoal() {
        return Commands.waitUntil(()-> arm.atGoal() && elevator.atGoal());
    }

    public Command finishScoring() {
        return Commands.waitUntil(()-> finishedScoring());
    }

    private boolean finishedScoring() {
        return isScoring() && atGoal();
    }

    private boolean isScoring() {
        return getState() == ArmManagerStates.SCORE_L2 || getState() == ArmManagerStates.SCORE_L3 || getState() == ArmManagerStates.SCORE_L4;
    }

    private boolean atGoal() {
        return arm.atGoal() && elevator.atGoal();
    }

    private static ArmManager instance;

    public static ArmManager getInstance() {
        if (instance == null)
            instance = new ArmManager(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
