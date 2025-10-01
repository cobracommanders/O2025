package frc.robot.subsystems.armManager.armScheduler;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
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
        super(ArmSchedulerState.READY);
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.visualization = new ArmSchedulerVisualization(driveWidth, driveHeight, intakeWidth, finalIntakeHeight);
    }

    private final double armLength = Units.inchesToMeters(24.5);
    private final double armWidth = Units.inchesToMeters(5.0);
    private final double handHeight = Units.inchesToMeters(5.0);

    private final double driveWidth = Units.inchesToMeters(35.0);
    private final double driveHeight = Units.inchesToMeters(6.5);

    private final double elevatorBaseHeight = Units.inchesToMeters(13.75);

    private final double intakeMinInterferenceUpHeightFromPivot = Units.inchesToMeters(15.5);
    private final double intakePivotHeight = Units.inchesToMeters(7.5);
    private final double intakeHeightOffset = Units.inchesToMeters(-4.0); // TODO ...
    private final double finalIntakeHeight = intakePivotHeight + intakeMinInterferenceUpHeightFromPivot + intakeHeightOffset;
    private final double intakeWidth = Units.inchesToMeters(21.25);

    private ArmSchedulerState assessState() {
        boolean canMoveElevatorInternally = !willArmHitIntakeOrDrivetrain(arm.getNormalizedPosition(), targetElevatorState.getPosition());
        boolean canMoveArmInternally = !willArmHitIntakeOrDrivetrain(targetArmState.getPosition(), elevator.getHeight());
        boolean willArmExtendOutOfFrame = willArmExtendOutOfFrame(targetArmState.getPosition());
        boolean isArmExtendingOutOfFrame = willArmExtendOutOfFrame(arm.getNormalizedPosition());

        if (willArmExtendOutOfFrame && isArmExtendingOutOfFrame) {
            return ArmSchedulerState.PARALLEL;
        }

        if (willArmExtendOutOfFrame && !elevatorAtPosition() && canMoveElevatorInternally) {
            return ArmSchedulerState.ELEVATOR_FIRST;
        } else if (isArmExtendingOutOfFrame && !armAtPosition() && canMoveArmInternally) {
            return ArmSchedulerState.ARM_FIRST;
        }

        if (canMoveElevatorInternally && canMoveArmInternally) {
            return ArmSchedulerState.PARALLEL;
        } else if (canMoveElevatorInternally) {
            return ArmSchedulerState.ELEVATOR_FIRST;
        } else if (canMoveArmInternally) {
            return ArmSchedulerState.ARM_FIRST;
        } else {
            // TODO remove for actual robot
            throw new IllegalStateException("Cannot move elevator or arm");
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

        if (willSwingOutOfFrame) {
            RobotScoringSide reefSide = AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
            return switch (reefSide) {
                case RIGHT -> ArmState.TRANSITION_OUTSIDE_FRAME_LEFT;
                case LEFT -> ArmState.TRANSITION_OUTSIDE_FRAME_RIGHT;
            };
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
            }
            case ARM_FIRST -> {
                arm.setState(getArmStateWithSwingDirection(targetArmState));
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

    private boolean isArmUp(double armAngle) {
        double angle = armAngle % 1.0;
        return angle > 0.0;
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
