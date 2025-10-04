package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.trailblazer.SwerveBase;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.MathHelpers;

import java.util.Map;

public class DriveSubsystem extends StateMachine<DriveStates> implements SwerveBase {
    private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
    private ChassisSpeeds autoSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds autoAlignSpeeds = new ChassisSpeeds();
    private ChassisSpeeds algaeAutoAlignSpeeds = new ChassisSpeeds();

    public final CommandSwerveDrivetrain drivetrain;
    private final SwerveDriveState drivetrainState = new SwerveDriveState();
    private double teleopSlowModePercent = 1.0;
    private double rawControllerXValue = 0.0;
    private double rawControllerYValue = 0.0;


    private double elevatorHeight;

    private final Timer timeSinceAutoSpeeds = new Timer();

    private static final double LEFT_X_DEADBAND = 0.05;
    private static final double LEFT_Y_DEADBAND = 0.05;
    private static final double RIGHT_X_DEADBAND = 0.15;

    private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE = InterpolatingDoubleTreeMap
            .ofEntries(Map.entry(0.0, 1.0), Map.entry(0.6, 1.0), Map.entry(1.4, 0.7));

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(DrivetrainConstants.maxSpeed * 0.03)
            .withRotationalDeadband(DrivetrainConstants.maxAngularRate * 0.03);

    private final SwerveRequest.FieldCentric autoDrive = new SwerveRequest.FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(0.05)
            .withRotationalDeadband(Units.degreesToRadians(2.0));

    public final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(DrivetrainConstants.maxSpeed * 0.03)
            .withRotationalDeadband(DrivetrainConstants.maxAngularRate * 0.03);

    private DriveSubsystem() {
        super(DriveStates.TELEOP);
        drivetrain = CommandSwerveDrivetrain.getInstance();

        timeSinceAutoSpeeds.start();
    }

    public Translation2d getControllerValues() {
        if (getState() != DriveStates.REEF_ALIGN_TELEOP || getState() != DriveStates.ALGAE_ALIGN_TELEOP) {
            return Translation2d.kZero;
        }

        return new Translation2d(
                ControllerHelpers.getJoystickMagnitude(
                        rawControllerXValue, rawControllerYValue, 2),
                new Rotation2d(rawControllerXValue, rawControllerYValue));
    }

    // public void scoringAlignmentRequest() {
    //   if (DriverStation.isAutonomous()) {
    //     normalDriveRequest();
    //   } else {
    //     setSnapToAngle(ReefPoses.getInstance().getNearestBranch().getRotation().getDegrees());
    //     setAutoAlignSpeeds();
    //     setStateFromRequest(DriveStates.REEF_ALIGN_TELEOP);
    //   }
    // }

    @Override
    protected DriveStates getNextState(DriveStates currentState) {
        return switch (currentState) {
            case AUTO, TELEOP -> FmsSubsystem.getInstance().isAutonomous() ? DriveStates.AUTO : DriveStates.TELEOP;
            case REEF_ALIGN_TELEOP -> AutoAlign.getInstance().isAlignedDebounced() ? DriveStates.TELEOP : DriveStates.REEF_ALIGN_TELEOP;
            case ALGAE_ALIGN_TELEOP ->
                    AutoAlign.getInstance().isAlignedDebounced() ? DriveStates.TELEOP : DriveStates.ALGAE_ALIGN_TELEOP;

        };
    }

    public void setTeleopSpeeds(double x, double y, double theta) {
        rawControllerXValue = x;
        rawControllerYValue = y;
        double leftY = -1.0
                * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(y, LEFT_Y_DEADBAND), 2.0);
        double leftX = -1.0
                * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(x, LEFT_X_DEADBAND), 2.0);
        double rightX = MathHelpers.signedExp(
                ControllerHelpers.deadbandJoystickValue(theta, RIGHT_X_DEADBAND), 1.3);

        DogLog.log("Swerve/LeftX", leftX);
        DogLog.log("Swerve/LeftY", leftY);
        DogLog.log("Swerve/RightX", rightX);
        Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
        double mappedX = mappedpose.getX();
        double mappedY = mappedpose.getY();

        teleopSpeeds = new ChassisSpeeds(
                // TODO if robot is driving backwards, get rid of the invert here
                mappedY * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
                mappedX * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
                rightX * DrivetrainConstants.maxAngularRate * teleopSlowModePercent);
    }

    public ChassisSpeeds getTeleopSpeeds() {
        return teleopSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    @Override
    public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
        autoSpeeds = speeds;
        timeSinceAutoSpeeds.reset();
    }

    private ChassisSpeeds calculateFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, drivetrainState.Pose.getRotation());
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getTagAlignSpeedsForAlign() {
        if (FmsSubsystem.getInstance().isAutonomous()) {
            return AutoAlign.getInstance().getTagAlignSpeeds();
        } else if (FmsSubsystem.getInstance().isRedAlliance()) {
            return flipSpeeds(AutoAlign.getInstance().getTagAlignSpeeds());
        } else {
            return AutoAlign.getInstance().getTagAlignSpeeds();
        }
    }

    @Override
    protected void collectInputs() {
        fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
        autoAlignSpeeds = getTagAlignSpeedsForAlign();
        algaeAutoAlignSpeeds = FmsSubsystem.getInstance().isRedAlliance() ? flipSpeeds(AutoAlign.getInstance().getAlgaeAlignSpeeds()) : AutoAlign.getInstance().getAlgaeAlignSpeeds();
        teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
        DogLog.log("Swerve/SlowModePercent", teleopSlowModePercent);
        DogLog.log("Swerve/Pose", drivetrain.getState().Pose);
    }

    public ChassisSpeeds flipSpeeds(ChassisSpeeds speeds) {
        // speeds = AutoAlign.getInstance().getAlgaeAlignSpeeds();
        return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    @Override
    public void periodic() {
        super.periodic();
        drivetrain.update();
        sendSwerveRequest(getState());
    }

    public void requestTeleop() {
        setStateFromRequest(DriveStates.TELEOP);
    }

    public void requestAuto() {
        setStateFromRequest(DriveStates.AUTO);
        setTeleopSpeeds(0.0, 0.0, 0.0);
    }

    public void requestReefAlign() {
        setStateFromRequest(DriveStates.REEF_ALIGN_TELEOP);
        setTeleopSpeeds(0.0, 0.0, 0.0);
    }

    public void requestAlgaeAlign() {
        setStateFromRequest(DriveStates.ALGAE_ALIGN_TELEOP);
        setTeleopSpeeds(0.0, 0.0, 0.0);
    }

    protected void sendSwerveRequest(DriveStates newState) {
        switch (newState) {
            case TELEOP -> {
                drivetrain.setControl(
                        drive
                                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                                .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                                .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                                .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
            }
            case AUTO, REEF_ALIGN_TELEOP -> {
                drivetrain.setControl(
                        autoDrive
                                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                                .withVelocityX(autoSpeeds.vxMetersPerSecond)
                                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                                .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                                .withDriveRequestType(DriveRequestType.Velocity));
            }

            case ALGAE_ALIGN_TELEOP -> {
                drivetrain.setControl(
                        autoDrive
                                .withVelocityX(algaeAutoAlignSpeeds.vxMetersPerSecond)
                                .withVelocityY(algaeAutoAlignSpeeds.vyMetersPerSecond)
                                .withRotationalRate(algaeAutoAlignSpeeds.omegaRadiansPerSecond)
                                .withDriveRequestType(DriveRequestType.Velocity));
            }
        }
    }

    public void setElevatorHeight(double height) {
        elevatorHeight = height;
    }

    private static DriveSubsystem instance;

    public static DriveSubsystem getInstance() {
        if (instance == null)
            instance = new DriveSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
