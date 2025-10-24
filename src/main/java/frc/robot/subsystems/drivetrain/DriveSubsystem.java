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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
    private final ChassisSpeeds algaeAutoAlignSpeeds = new ChassisSpeeds();

    public final TunerConstants.TunerSwerveDrivetrain drivetrain = new TunerConstants.TunerSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
    );

    private final SwerveDriveState drivetrainState = drivetrain.getState();

    private double teleopSlowModePercent = 1.0;

    private double elevatorHeight;

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
//            .withDeadband(0.05)
//            .withRotationalDeadband(Units.degreesToRadians(2.0))
            ;

    private boolean hasAppliedOperatorPerspective = false;

    private DriveSubsystem() {
        super(DriveStates.TELEOP, "DriveSubsystem");
    }

    @Override
    protected DriveStates getNextState(DriveStates currentState) {
        DriveStates nextState = currentState;
        switch (currentState) {
            case AUTO, TELEOP ->
                    nextState = FmsSubsystem.getInstance().isAutonomous() ? DriveStates.AUTO : DriveStates.TELEOP;
            case REEF_ALIGN_TELEOP -> { /* Await Control */ }
        }

        return nextState;
    }

    public void setTeleopSpeeds(double x, double y, double theta) {
        double leftY = -1.0
                * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(y, LEFT_Y_DEADBAND), 2.0);
        double leftX = -1.0
                * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(x, LEFT_X_DEADBAND), 2.0);
        double rightX = MathHelpers.signedExp(
                ControllerHelpers.deadbandJoystickValue(theta, RIGHT_X_DEADBAND), 1.3);

        DogLog.log("Swerve/LeftX", leftX);
        DogLog.log("Swerve/LeftY", leftY);
        DogLog.log("Swerve/RightX", rightX);

        Translation2d mappedPose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
        double mappedX = mappedPose.getX();
        double mappedY = mappedPose.getY();

        teleopSpeeds = new ChassisSpeeds(
                mappedY * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
                mappedX * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
                rightX * DrivetrainConstants.maxAngularRate * teleopSlowModePercent
        );
    }

    public ChassisSpeeds getTeleopSpeeds() {
        return teleopSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation());
    }

    @Override
    public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
        autoSpeeds = speeds;
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    protected void collectInputs() {
        teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);

        DogLog.log("Swerve/AutoSpeeds", autoSpeeds);
        DogLog.log("Swerve/TeleopSpeeds", teleopSpeeds);
        DogLog.log("Swerve/FieldRelativeSpeeds", getFieldRelativeSpeeds());
        DogLog.log("Swerve/SlowModePercent", teleopSlowModePercent);
        DogLog.log("Swerve/Pose", drivetrain.getState().Pose);
    }

    public ChassisSpeeds flipSpeeds(ChassisSpeeds speeds) {
        return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasAppliedOperatorPerspective || FmsSubsystem.getInstance().isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                drivetrain.setOperatorPerspectiveForward(allianceColor == DriverStation.Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                hasAppliedOperatorPerspective = true;
            });
        }

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
        }
    }

    public Rotation2d getRawYaw() {
        return drivetrainState.RawHeading;
    }

    public double getAngularVelocityYWorld() {
        return drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble();
    }

    public void setYaw(Rotation2d rotation) {
        drivetrain.resetRotation(rotation);
    }

    public void setYawFromFMS() {
        setYaw(FmsSubsystem.getInstance().isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
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
