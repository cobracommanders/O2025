package frc.robot.subsystems.armManager.armScheduler;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.arm.ArmState;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.armManager.hand.HandState;

import static java.lang.Math.abs;


public class ArmScheduler extends StateMachine<ArmSchedulerState> {
    private final Arm arm;
    private final Elevator elevator;
    private final Hand hand;
    private final ArmSchedulerVisualization visualization;

    private ArmState targetArmState;
    private ElevatorState targetElevatorState;
    private HandState targetHandState;

    public ArmScheduler(
            Arm arm,
            Elevator elevator,
            Hand hand
    ) {
        super(ArmSchedulerState.READY, "ArmScheduler");
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.visualization = new ArmSchedulerVisualization(driveWidth, driveHeight, intakeWidth, finalIntakeHeight);
    }

    private final double armLength = Units.inchesToMeters(24.5);
    private final double armWidth = Units.inchesToMeters(5.0); // Left-right width of the arm when viewed from the front
    private final double handHeight = Units.inchesToMeters(5.0); // Used for algae floor intake, the length of arm that can contact the intake when the arm is horizontal

    private final double driveWidth = Units.inchesToMeters(35.0);
    private final double driveHeight = Units.inchesToMeters(6.5);

    private final double elevatorBaseHeight = Units.inchesToMeters(13.75);

    private final double intakeMinInterferenceUpHeightFromPivot = Units.inchesToMeters(15.5);
    private final double intakePivotHeight = Units.inchesToMeters(7.5);
    private final double intakeHeightOffset = Units.inchesToMeters(-5.0);
    private final double finalIntakeHeight = intakePivotHeight + intakeMinInterferenceUpHeightFromPivot + intakeHeightOffset;
    private final double intakeWidth = Units.inchesToMeters(21.25);

    private ArmSchedulerState assessState() {
        boolean canMoveElevatorInternally = !willArmHitIntakeOrDrivetrain(arm.getNormalizedPosition(), targetElevatorState.getPosition());
        boolean canMoveArmInternally = !willArmHitIntakeOrDrivetrain(targetArmState.getPosition(), elevator.getHeight());

        // Ideally no longer needed with new coral autoalign

//        boolean willArmExtendOutOfFrame = willArmExtendOutOfFrame(targetArmState.getPosition());
//        boolean isArmExtendingOutOfFrame = willArmExtendOutOfFrame(arm.getNormalizedPosition());

//        if (willArmExtendOutOfFrame && isArmExtendingOutOfFrame) {
//            return ArmSchedulerState.PARALLEL;
//        }
//
//        if (willArmExtendOutOfFrame && !elevatorAtPosition() && canMoveElevatorInternally) {
//            return ArmSchedulerState.ELEVATOR_FIRST;
//        } else if (isArmExtendingOutOfFrame && !armAtPosition() && canMoveArmInternally) {
//            return ArmSchedulerState.ARM_FIRST;
//        }

        if (canMoveElevatorInternally && canMoveArmInternally) {
            return ArmSchedulerState.PARALLEL;
        } else if (canMoveElevatorInternally) {
            return ArmSchedulerState.ELEVATOR_FIRST;
        } else if (canMoveArmInternally) {
            return ArmSchedulerState.ARM_FIRST;
        } else {
            return getState();
            // TODO remove for actual robot
            //throw new IllegalStateException("Cannot move elevator or arm");
        }
    }

    private boolean willArmHitIntakeOrDrivetrain(double armPosition, double elevatorPosition) {
        return willArmHitIntake(armPosition, elevatorPosition) || willArmHitDrivetrain(armPosition, elevatorPosition);
    }

    private boolean willArmHitIntake(double armPosition, double elevatorPosition) {
        Coordinate[] coordinates = getArmCoordinates(armPosition, elevatorPosition);
        Coordinate coordinate1 = coordinates[0];
        Coordinate coordinate2 = coordinates[1];
        return (coordinate1.y() < finalIntakeHeight && abs(coordinate1.x() - intakeWidth) > handHeight) || (coordinate2.y() < finalIntakeHeight && abs(coordinate2.x() - intakeWidth) > handHeight);
    }

    private boolean willArmHitDrivetrain(double armPosition, double elevatorPosition) {
        if (DriverStation.isAutonomous()) return false;
        Coordinate[] coordinates = getArmCoordinates(armPosition, elevatorPosition);
        Coordinate coordinate1 = coordinates[0];
        Coordinate coordinate2 = coordinates[1];
        return (coordinate1.y() < driveHeight && abs(coordinate1.x() - driveWidth) > handHeight) || (coordinate2.y() < driveHeight && abs(coordinate2.x() - driveWidth) > handHeight);
    }

    private boolean willArmExtendOutOfFrame(double armPosition) {
        Coordinate[] coordinates = getArmCoordinates(armPosition, 0.0);
        Coordinate coordinate1 = coordinates[0];
        Coordinate coordinate2 = coordinates[1];
        return abs(coordinate1.x()) > driveWidth / 2.0 || abs(coordinate2.x()) > driveWidth / 2.0;
    }

    /**
     * Checks if the arm will swing through the outside of the frame perimiter, but starts and ends within the frame.
     */
    private boolean willArmSwingThroughOutsideFrame(double armPosition, double targetArmPosition) {
        return !willArmExtendOutOfFrame(armPosition) && !willArmExtendOutOfFrame(targetArmPosition) && (isArmUp(armPosition) != isArmUp(targetArmPosition));
    }

    private Coordinate[] getArmCoordinates(double armPosition, double elevatorPosition) {
        var armPositionRadians = Units.rotationsToRadians(armPosition);

        // Calculate the center of the arm
        var armEndCenterY = (Math.sin(armPositionRadians) * armLength) + elevatorBaseHeight + elevatorPosition;
        var armEndCenterX = Math.cos(armPositionRadians) * armLength;

        // Calculate the end points of the arm
        var armY1 = Math.sin(armPositionRadians + Math.PI / 2.0) * (armWidth / 2) + armEndCenterY;
        var armX1 = Math.cos(armPositionRadians + Math.PI / 2.0) * (armWidth / 2) + armEndCenterX;
        var armY2 = Math.sin(armPositionRadians - Math.PI / 2.0) * (armWidth / 2) + armEndCenterY;
        var armX2 = Math.cos(armPositionRadians - Math.PI / 2.0) * (armWidth / 2) + armEndCenterX;

        return new Coordinate[]{new Coordinate(armX1, armY1), new Coordinate(armX2, armY2)};
    }

    private record Coordinate(double x, double y) {
    }

    @Override
    protected ArmSchedulerState getNextState(ArmSchedulerState currentState) {
        ArmSchedulerState nextState = currentState;

        switch (currentState) {
            case NEW_COMMAND, PARALLEL, ELEVATOR_FIRST, ARM_FIRST -> {
                nextState = assessState();

                if (atPosition()) {
                    nextState = ArmSchedulerState.READY;
                }
            }
            case READY -> { /* Await Control */ }
        }

        return nextState;
    }

    /**
     * Gets the arm state that ensures that if the arm will swing through the outside of the frame perimiter, it will swing in the correct direction away from the reef.
     */
    private ArmState getArmStateWithSwingDirection(ArmState state) {
        boolean willSwingOutOfFrame = willArmSwingThroughOutsideFrame(arm.getNormalizedPosition(), state.getPosition());
        RobotScoringSide reefSide = AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
        // return switch (reefSide) {
        //        case RIGHT -> ArmState.TRANSITION_OUTSIDE_FRAME_LEFT;
        //        case LEFT -> ArmState.TRANSITION_OUTSIDE_FRAME_RIGHT;
        // };
        if (willSwingOutOfFrame) {
           //RobotScoringSide reefSide = AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
        //    return switch (reefSide) {
        //        case RIGHT -> ArmState.TRANSITION_OUTSIDE_FRAME_LEFT;
        //        case LEFT -> ArmState.TRANSITION_OUTSIDE_FRAME_RIGHT;
        //    };
            return state;
        } else {
            return state;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        // We want constant updating, so this goes in periodic instead of after Transition()
        switch (getState()) {
            case NEW_COMMAND -> {
            }
            case PARALLEL -> {
                arm.setState(getArmStateWithSwingDirection(targetArmState));
                elevator.setState(targetElevatorState);
            }
            case ELEVATOR_FIRST -> {
                elevator.setState(targetElevatorState);
                double armPosition = getNearestArmPositionWithoutCollision(elevator.getHeight(), targetArmState.getPosition(), arm.getNormalizedPosition());
                arm.setCustom(armPosition);
            }
            case ARM_FIRST -> {
                arm.setState(getArmStateWithSwingDirection(targetArmState));
                double elevatorPosition = getNearestElevatorHeightWithoutArmCollision(arm.getNormalizedPosition(), targetElevatorState.getPosition());
                elevator.setCustom(elevatorPosition);
            }
            case READY -> {
                // Clear target state
                targetArmState = null;
                targetHandState = null;
                targetElevatorState = null;
            }
        }

        if (targetHandState != null) {
            hand.setState(targetHandState);
        }
    }

    /**
     * Gets the elevator position that will be closest to the desired position while not hitting the intake or drivetrain with the arm.
     */
    public double getNearestElevatorHeightWithoutArmCollision(double armPosition, double targetElevatorPosition) {
        // This method will only be called in ELEVATOR_FIRST mode, so we know the arm would hit the intake or drivetrain if the elevator moved all the way, therefore move just above the intake to be safe

        boolean useIntakeHeight = true;
        // if (armPosition >= -0.05){
        //     useIntakeHeight = false;
        // }

        if(useIntakeHeight == true){
            double desiredHeight = finalIntakeHeight + (armWidth / 2.0) + Units.inchesToMeters(1.0);

            double minElevatorPositionForArm = getElevatorPositionForDesiredArmHeight(armPosition, desiredHeight);

        // No need to go below the target position
            return Math.max(targetElevatorPosition, minElevatorPositionForArm);
        } else {
            double desiredHeightWithoutIntake = (armWidth / 2.0) + Units.inchesToMeters(1.0);

            double minElevatorPositionForArmWithoutIntake = getElevatorPositionForDesiredArmHeight(armPosition, desiredHeightWithoutIntake);

            return Math.max(targetElevatorPosition, minElevatorPositionForArmWithoutIntake);
        }
    }

    /**
     * Gets the arm position that will be closest to the desired position while not hitting the intake or drivetrain.
     */
    public double getNearestArmPositionWithoutCollision(double elevatorPosition, double targetArmPosition, double armPosition) {
        // This method will only be called in ARM_FIRST mode, so we know the arm would hit the intake or drivetrain if it moved all the way, therefore move just above the intake to be safe
        double desiredHeight = finalIntakeHeight + (armWidth / 2.0) + Units.inchesToMeters(1.0);

        double solutionOne = Arm.normalizePosition(getArmPositionForDesiredArmHeight(elevatorPosition, desiredHeight));
        double solutionTwo = Arm.normalizePosition(ArmState.invertPosition(solutionOne));

        double solutionOneDiff = Math.abs(solutionOne - armPosition);
        double solutionTwoDiff = Math.abs(solutionTwo - armPosition);

        return solutionOneDiff < solutionTwoDiff ? solutionOne : solutionTwo;
    }

    /**
     * Gets the elevator position that will put the center of the end of the arm at the desired height from the floor.
     */
    public double getElevatorPositionForDesiredArmHeight(double armPosition, double desiredArmHeight) {
        double armPositionRadians = Units.rotationsToRadians(armPosition);
        double currentArmHeight = Math.sin(armPositionRadians) * armLength;

        return desiredArmHeight - currentArmHeight - elevatorBaseHeight;
    }

    /**
     * Gets an arm position that will put the center of the end of the arm at the desired height from the floor.
     */
    public double getArmPositionForDesiredArmHeight(double elevatorHeight, double desiredArmHeight) {
        double neededArmHeight = desiredArmHeight - elevatorHeight - elevatorBaseHeight;
        return Units.radiansToRotations(Math.asin(neededArmHeight / armLength));
    }

    @Override
    protected void collectInputs() {
        DogLog.log("ArmScheduler/atPosition", atPosition());
        DogLog.log("ArmScheduler/armAngle", arm.getNormalizedPosition());
        DogLog.log("ArmScheduler/elevatorHeight", elevator.getHeight());
        DogLog.log("ArmScheduler/armUp", isArmUp(arm.getNormalizedPosition()));
        SmartDashboard.putData("ArmScheduler/ArmViz", visualization.getMechanism2d());
        DogLog.log("ArmScheduler/armHittingIntake", willArmHitIntake(arm.getNormalizedPosition(), elevator.getHeight()));
        DogLog.log("ArmScheduler/armHittingDrivetrain", willArmHitDrivetrain(arm.getNormalizedPosition(), elevator.getHeight()));
        DogLog.log("ArmScheduler/armExtendingOutOfFrame", willArmExtendOutOfFrame(arm.getNormalizedPosition()));
        Coordinate[] coordinates = getArmCoordinates(arm.getNormalizedPosition(), elevator.getHeight());

        DogLog.log("ArmScheduler/armRight", isArmRight());
        

        Coordinate coordinate1 = coordinates[0];
        Coordinate coordinate2 = coordinates[1];
        visualization.drawArm(
                elevatorBaseHeight + elevator.getHeight(),
                coordinate1.x(),
                coordinate1.y(),
                coordinate2.x(),
                coordinate2.y());
        visualization.drawIntake(intakeWidth, finalIntakeHeight);
    }

    public boolean isArmUp() {
        return isArmUp(arm.getNormalizedPosition());
    }

    private boolean isArmUp(double armAngle) {
        double angle = armAngle % 1.0;
        return angle > 0.0;
    }

    private boolean isArmRight() {
        return MathUtil.isNear(arm.getNormalizedPosition(), 0.0, 0.25);
    }

    public void scheduleStates(ArmState armState, ElevatorState elevatorState, HandState handState) {
        this.targetArmState = armState;
        this.targetElevatorState = elevatorState;
        this.targetHandState = handState;

        setStateFromRequest(ArmSchedulerState.NEW_COMMAND);
    }

    /**
     * Checks that both the Arm and Elevator are at position in the final state.
     */
    private boolean atPosition() {
        return elevatorAtPosition() && armAtPosition();
    }

    private boolean armAtPosition() {
        return arm.atGoal() && arm.getState() == targetArmState;
    }

    private boolean elevatorAtPosition() {
        return elevator.atGoal() && elevator.getState() == targetElevatorState;
    }

    public boolean isReady() {
        return getState() == ArmSchedulerState.READY;
    }
}
