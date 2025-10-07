package frc.robot.trailblazer;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.trailblazer.constraints.AutoConstraintCalculator;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.trailblazer.followers.PathFollower;
import frc.robot.trailblazer.followers.PidPathFollower;
import frc.robot.trailblazer.trackers.PathTracker;
import frc.robot.trailblazer.trackers.pure_pursuit.PurePursuitPathTracker;
import frc.robot.util.TimestampedChassisSpeeds;

public class Trailblazer {
    private final SwerveBase swerve;
    private final LocalizationBase localization;

    private final PathTracker pathTracker = new PurePursuitPathTracker(false, true);
    private final PathFollower pathFollower =
            new PidPathFollower(new PIDController(3.7, 0, 0.0), new PIDController(4.0, 0, 0.3));
    private int previousAutoPointIndex = -1;
    private TimestampedChassisSpeeds previousSpeeds = new TimestampedChassisSpeeds(0);

    public Trailblazer(SwerveBase swerve, LocalizationBase localization) {
        this.swerve = swerve;
        this.localization = localization;
    }

    public Command followSegment(AutoSegment segment) {
        return followSegment(segment, true);
    }

    public Command followSegment(AutoSegment segment, boolean shouldEnd) {
        TrailblazerPathLogger.logSegment(segment);
        var command =
                Commands.runOnce(
                                () -> {
                                    pathTracker.resetAndSetPoints(segment.points);
                                    previousAutoPointIndex = -1;
                                    DogLog.log(
                                            "Autos/Trailblazer/CurrentSegment/InitialPoints",
                                            segment.points.stream()
                                                    .map(point -> point.poseSupplier.get())
                                                    .toArray(Pose2d[]::new));
                                })
                        .alongWith(
                                Commands.run(
                                        () -> {
                                            pathTracker.updateRobotState(
                                                    localization.getPose(), swerve.getFieldRelativeSpeeds());
                                            var currentAutoPointIndex = pathTracker.getCurrentPointIndex();
                                            var currentAutoPoint = segment.points.get(currentAutoPointIndex);
                                            double distanceToSegmentEnd =
                                                    segment.getRemainingDistance(
                                                            localization.getPose(), currentAutoPointIndex);

                                            var constrainedVelocityGoal =
                                                    getSwerveSetpoint(
                                                            currentAutoPoint, segment.defaultConstraints, distanceToSegmentEnd);
                                            swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                                            DogLog.log(
                                                    "Autos/Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
                                            if (previousAutoPointIndex != currentAutoPointIndex) {
                                                // Currently tracked point has changed, trigger side effects

                                                // Each of the points in (previous, current]
                                                var pointsToRunSideEffectsFor =
                                                        segment.points.subList(
                                                                previousAutoPointIndex + 1, currentAutoPointIndex + 1);
                                                for (var passedPoint : pointsToRunSideEffectsFor) {
                                                    DogLog.log(
                                                            "Autos/Trailblazer/Tracker/CommandTriggered",
                                                            passedPoint.command.getName());
                                                    passedPoint.command.schedule();
                                                }
                                                previousAutoPointIndex = currentAutoPointIndex;
                                            }
                                        },
                                        swerve))
                        .withName("FollowSegmentIndefinitely");

        if (shouldEnd) {
            return command
                    .until(
                            () -> segment.isFinished(localization.getPose(), pathTracker.getCurrentPointIndex()))
                    .andThen(
                            Commands.runOnce(
                                    () -> {
                                        swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds());
                                    }))
                    .withName("FollowSegmentUntilFinished");
        }

        return command;
    }

    private ChassisSpeeds getSwerveSetpoint(
            AutoPoint point, AutoConstraintOptions segmentConstraints, double distanceToSegmentEnd) {
        if (previousSpeeds.timestampSeconds == 0) {
            previousSpeeds = new TimestampedChassisSpeeds(Timer.getFPGATimestamp() - 0.02);
        }

        Pose2d robotPose = localization.getPose();
        Pose2d targetPose = pathTracker.getTargetPose();
        var rawVelocityGoal = new TimestampedChassisSpeeds(pathFollower.calculateSpeeds(robotPose, targetPose));

        // Get point-specific constraints if applicable, otherwise use the constraints of the full segment
        var constraints = point.constraints.orElse(segmentConstraints);

        // Constrain the output of the path follower based on the current constraints
        var constrainedVelocityGoal = AutoConstraintCalculator.constrainVelocityGoal(rawVelocityGoal, previousSpeeds, constraints, distanceToSegmentEnd);

        // Update previous speeds for acceleration calculations
        previousSpeeds = constrainedVelocityGoal;

        DogLog.log("Autos/Trailblazer/Constraints/Linear Velocity", constraints.maxLinearVelocity());
        DogLog.log("Autos/Trailblazer/Constraints/Linear Acceleration", constraints.maxLinearAcceleration());

        DogLog.log("Autos/Trailblazer/Constraints/Angular Velocity", constraints.maxAngularVelocity());
        DogLog.log("Autos/Trailblazer/Constraints/Angular Acceleration", constraints.maxAngularAcceleration());

        DogLog.log("Autos/Trailblazer/Tracker/TargetPose", targetPose);

        DogLog.log("Autos/Trailblazer/Follower/Initial Goal", rawVelocityGoal);
        DogLog.log("Autos/Trailblazer/Follower/Constrained Goal", constrainedVelocityGoal);

        return constrainedVelocityGoal;
    }
}
