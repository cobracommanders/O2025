package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.LocalizationBase;
import frc.robot.util.MathHelpers;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;

public class LocalizationSubsystem extends StateMachine<LocalizationStates> implements LocalizationBase {
    private static final Vector<N3> MT1_VISION_STD_DEVS = VecBuilder.fill(
            VisionConstants.xyStandardDeviation, // xy standard deviation
            VisionConstants.xyStandardDeviation, // xy standard deviation
            VisionConstants.thetaStandardDeviation // theta standard deviation
    );
    private static final Vector<N3> MT2_VISION_STD_DEVS = VecBuilder.fill(
            VisionConstants.xyStandardDeviation, //xy standard deviation
            VisionConstants.thetaStandardDeviation, //xy standard deviation
            Double.MAX_VALUE);
    private final VisionSubsystem vision;
    private final DriveSubsystem swerve;
    private final DriveSubsystem drivetrain;
    private Pose2d robotPose = Pose2d.kZero;

    private LocalizationSubsystem() {
        super(LocalizationStates.DEFAULT_STATE, "LocalizationSubsystem");
        this.swerve = DriveSubsystem.getInstance();
        this.drivetrain = DriveSubsystem.getInstance();
        this.vision = VisionSubsystem.getInstance();

        if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
            SmartDashboard.putData(
                    "FieldCalibration/ResetGyroTo180",
                    Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
            SmartDashboard.putData(
                    "FieldCalibration/ResetGyroTo0",
                    Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
            SmartDashboard.putData(
                    "FieldCalibration/ResetGyroTo90",
                    Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
            SmartDashboard.putData(
                    "FieldCalibration/ResetGyroTo270",
                    Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
        }
    }

    @Override
    protected void collectInputs() {
        vision
                .getLeftFrontTagResult()
                .or(vision::getLeftBackTagResult)
                .ifPresent(this::ingestTagResult);
        vision
                .getRightFrontTagResult()
                .or(vision::getRightBackTagResult)
                .ifPresent(this::ingestTagResult);
        vision.getGamePieceTagResult().ifPresent(this::ingestTagResult);
        robotPose = swerve.drivetrain.getState().Pose;
        vision.setEstimatedPoseAngle(robotPose.getRotation().getDegrees());
    }

    @Override
    public Pose2d getPose() {
        return robotPose;
    }

    public Pose2d getPose(double timestamp) {
        var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
        return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
    }

    public Pose2d getLookaheadPose(double lookahead) {
        return MathHelpers.poseLookahead(getPose(), swerve.getTeleopSpeeds(), lookahead);
    }

    @Override
    public void periodic() {
        super.periodic();

        DogLog.log("Localization/EstimatedPose", getPose());
    }

    private void ingestTagResult(TagResult result) {
        var visionPose = result.pose();

        if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
            resetPose(visionPose);
        }

        swerve.drivetrain.addVisionMeasurement(
                visionPose, Utils.fpgaToCurrentTime(result.timestamp()), result.standardDevs());
        DogLog.log("Localization/VisionPose", visionPose);
    }

    private void resetGyro(Rotation2d gyroAngle) {
        drivetrain.setYaw(gyroAngle);
        swerve.drivetrain.resetRotation(gyroAngle);
    }

    public void resetPose(Pose2d estimatedPose) {
        // Reset the gyro when requested in teleop
        // Otherwise, if we are in auto, only reset it if we aren't already at the
        // correct heading
        if (DriverStation.isTeleop()
                || !MathUtil.isNear(
                estimatedPose.getRotation().getDegrees(), drivetrain.getRawYaw().getDegrees(), 1.5, -180,
                180)) {
            drivetrain.setYaw(estimatedPose.getRotation());
        }

        swerve.drivetrain.resetPose(estimatedPose);
    }

    public Command getZeroCommand() {
        return Commands.runOnce(
                () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.getInstance().isRedAlliance() ? 180 : 0))));
    }

    private static LocalizationSubsystem instance;

    public static LocalizationSubsystem getInstance() {
        if (instance == null)
            instance = new LocalizationSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}