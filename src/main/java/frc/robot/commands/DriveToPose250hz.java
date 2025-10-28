package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

import java.util.function.Supplier;

public class DriveToPose250hz extends Command implements SwerveRequest {
    private final double PERIOD = 0.004;
    private final ProfiledPIDController driveController;
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    5.0,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            Units.degreesToRadians(360),
                            Units.degreesToRadians(720)),
                    PERIOD
            );
    private final DriveSubsystem driveSubsystem;
    private final double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private final double distanceFromEndToCompleteRotation = 0.75;
    private final Supplier<Pose2d> targetLocationSupplier;
    private Pose2d targetPosition = null;
    private final PoseErrorTolerance tolerance;

    private final FieldCentric fieldCentric = new FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private double totalDistance = 0.0;
    private Rotation2d initialRotation = Rotation2d.kZero;

    public DriveToPose250hz(
            DriveSubsystem driveSubsystem,
            Supplier<Pose2d> targetLocationSupplier,
            AutoConstraintOptions constraints,
            PoseErrorTolerance tolerance
    ) {
        this.driveSubsystem = driveSubsystem;
        this.targetLocationSupplier = targetLocationSupplier;
        this.driveController =
                new ProfiledPIDController(
                        7.0,
                        0.0,
                        0.02,
                        new TrapezoidProfile.Constraints(
                                constraints.maxLinearVelocity(),
                                constraints.maxLinearAcceleration()),
                        PERIOD
                );
        this.tolerance = tolerance;
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = LocalizationSubsystem.getInstance().getPose();

        this.targetPosition = targetLocationSupplier.get();
        this.totalDistance = targetPosition.getTranslation().getDistance(currentPose.getTranslation());
        this.initialRotation = currentPose.getRotation();


        double currentVelocity = Math.min(
                0.0,
                -Math.hypot(
                        driveSubsystem.getFieldRelativeSpeeds().vxMetersPerSecond,
                        driveSubsystem.getFieldRelativeSpeeds().vyMetersPerSecond
                ));

        driveController.reset(
                currentPose.getTranslation().getDistance(targetPosition.getTranslation()),
                currentVelocity
        );

        thetaController.reset(
                currentPose.getRotation().getRadians(),
                driveSubsystem.getFieldRelativeSpeeds().omegaRadiansPerSecond
        );

        driveSubsystem.requestDriveToPose250hz(this);
    }

    @Override
    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        double xError = targetPosition.getX() - parameters.currentPose.getX();
        double yError = targetPosition.getY() - parameters.currentPose.getY();
        double currentDistance = Math.hypot(xError, yError);

        double ffScalar = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

        double linearVelocity = driveController.getSetpoint().velocity * ffScalar + driveController.calculate(currentDistance, 0.0);
        if (currentDistance < tolerance.linearErrorTolerance()) linearVelocity = 0.0;
        DogLog.log("DriveToPose250hz/LinearVelocity", linearVelocity);

        double targetRotationRadians = Units.rotationsToRadians(
                MathUtil.interpolate(
                        MathUtil.inputModulus(initialRotation.getRotations(), -0.5, 0.5),
                        MathUtil.inputModulus(targetPosition.getRotation().getRotations(), -0.5, 0.5),
                        (totalDistance - (currentDistance - distanceFromEndToCompleteRotation)) / totalDistance)
        );
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScalar + thetaController.calculate(parameters.currentPose.getRotation().getRadians(), targetRotationRadians);
        double thetaErrorAbs = Math.abs(parameters.currentPose.getRotation().minus(targetPosition.getRotation()).getRadians());
        if (thetaErrorAbs < Units.degreesToRadians(tolerance.angularErrorTolerance())) thetaVelocity = 0.0;


        double magnitude = Math.max(Math.hypot(xError, yError), 1.0E-6); // Ensure no division by zero
        double cos = xError / magnitude;
        double sin = yError / magnitude;

        double vx = linearVelocity * cos;
        double vy = linearVelocity * sin;

        return fieldCentric
                .withVelocityX(-vx) // Negate velocity because pid controller is trying to go from a given distance (always positive) to zero and is therefore negative
                .withVelocityY(-vy)
                .withRotationalRate(thetaVelocity)
                .apply(parameters, modulesToApply);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.requestTeleop();
    }

    @Override
    public boolean isFinished() {
        return targetPosition == null || tolerance.atPose(targetPosition, LocalizationSubsystem.getInstance().getPose());
    }
}