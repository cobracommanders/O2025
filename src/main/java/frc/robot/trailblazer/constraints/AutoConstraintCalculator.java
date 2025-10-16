package frc.robot.trailblazer.constraints;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.PolarChassisSpeeds;
import frc.robot.util.TimestampedChassisSpeeds;

public class AutoConstraintCalculator {
    public static TimestampedChassisSpeeds constrainVelocityGoal(
            TimestampedChassisSpeeds inputSpeeds,
            TimestampedChassisSpeeds previousSpeeds,
            AutoConstraintOptions options,
            double distanceToSegmentEnd) {
        ChassisSpeeds constrainedSpeeds =
                constrainVelocityGoal(inputSpeeds, previousSpeeds, options);

        double newLinearVelocityForSafeDeceleration =
                getAccelerationBasedVelocityConstraint(
                        constrainedSpeeds, distanceToSegmentEnd, options);

        constrainedSpeeds =
                constrainLinearVelocity(
                        constrainedSpeeds,
                        options.withMaxLinearVelocity(newLinearVelocityForSafeDeceleration));

        return new TimestampedChassisSpeeds(constrainedSpeeds, inputSpeeds.timestampSeconds);
    }

    public static ChassisSpeeds constrainVelocityGoal(
            TimestampedChassisSpeeds inputSpeeds,
            TimestampedChassisSpeeds previousSpeeds,
            AutoConstraintOptions options) {
        var limitedSpeeds = inputSpeeds;

        if (options.maxLinearVelocity() != 0) {
            limitedSpeeds =
                    new TimestampedChassisSpeeds(
                            constrainLinearVelocity(limitedSpeeds, options),
                            limitedSpeeds.timestampSeconds);
        }

        if (options.maxAngularVelocity() != 0) {
            limitedSpeeds =
                    new TimestampedChassisSpeeds(
                            constrainRotationalVelocity(limitedSpeeds, options),
                            limitedSpeeds.timestampSeconds);
        }

        if (options.maxLinearAcceleration() != 0) {
            limitedSpeeds =
                    new TimestampedChassisSpeeds(
                            constrainLinearAcceleration(limitedSpeeds, previousSpeeds, options),
                            limitedSpeeds.timestampSeconds);
        }

        if (options.maxAngularAcceleration() != 0) {
            limitedSpeeds =
                    new TimestampedChassisSpeeds(
                            constrainRotationalAcceleration(limitedSpeeds, previousSpeeds, options),
                            limitedSpeeds.timestampSeconds);
        }

        return limitedSpeeds;
    }

    public static ChassisSpeeds constrainLinearVelocity(
            ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
        double currentLinearVelocity =
                hypot(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);

        if (currentLinearVelocity > options.maxLinearVelocity()) {
            return new PolarChassisSpeeds(
                    options.maxLinearVelocity(),
                    new Rotation2d(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond),
                    inputSpeeds.omegaRadiansPerSecond);
        } else {
            return inputSpeeds;
        }
    }

    private static ChassisSpeeds constrainRotationalVelocity(
            ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
        double currentAngularVelocity = inputSpeeds.omegaRadiansPerSecond;

        if (currentAngularVelocity > options.maxAngularVelocity()) {
            return new ChassisSpeeds(
                    inputSpeeds.vxMetersPerSecond,
                    inputSpeeds.vyMetersPerSecond,
                    options.maxAngularVelocity());
        } else {
            return inputSpeeds;
        }
    }

    private static ChassisSpeeds constrainLinearAcceleration(
            TimestampedChassisSpeeds currentSpeeds,
            TimestampedChassisSpeeds previousSpeeds,
            AutoConstraintOptions options) {

        double inputTotalSpeed =
                hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double previousTotalSpeed =
                hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);

        double currentLinearAcceleration =
                abs(inputTotalSpeed - previousTotalSpeed)
                        / currentSpeeds.timestampDifference(previousSpeeds);

        if (currentLinearAcceleration > options.maxLinearAcceleration()) {
            double distanceAtMaxAcceleration =
                    options.maxLinearAcceleration()
                            * currentSpeeds.timestampDifference(previousSpeeds);
            double limitedLinearSpeed =
                    previousTotalSpeed
                            + Math.copySign(
                                    distanceAtMaxAcceleration,
                                    inputTotalSpeed - previousTotalSpeed);
            return new PolarChassisSpeeds(
                    limitedLinearSpeed,
                    new Rotation2d(
                            currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                    currentSpeeds.omegaRadiansPerSecond);
        } else {
            return currentSpeeds;
        }
    }

    private static ChassisSpeeds constrainRotationalAcceleration(
            TimestampedChassisSpeeds currentSpeeds,
            TimestampedChassisSpeeds previousSpeeds,
            AutoConstraintOptions options) {

        double currentAngularSpeed = currentSpeeds.omegaRadiansPerSecond;
        double previousAngularSpeed = previousSpeeds.omegaRadiansPerSecond;

        double currentAngularAcceleration =
                abs(currentAngularSpeed - previousAngularSpeed)
                        / currentSpeeds.timestampDifference(previousSpeeds);

        if (currentAngularAcceleration > options.maxAngularAcceleration()) {
            double deltaAtMaxAcceleration =
                    options.maxAngularAcceleration()
                            * currentSpeeds.timestampDifference(previousSpeeds);
            double limitedAngularSpeed =
                    previousAngularSpeed
                            + Math.copySign(
                                    deltaAtMaxAcceleration,
                                    currentAngularSpeed - previousAngularSpeed);
            return new ChassisSpeeds(
                    currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond,
                    limitedAngularSpeed);
        }
        return currentSpeeds;
    }

    public static double getAccelerationBasedVelocityConstraint(
            ChassisSpeeds currentSpeeds,
            double distanceToSegmentEnd,
            AutoConstraintOptions options) {
        double currentVelocity =
                hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double decelerationDistance =
                Math.pow(currentVelocity, 2) / (2.0 * options.maxLinearAcceleration());

        if (distanceToSegmentEnd > decelerationDistance) {
            return currentVelocity;
        }

        // Limit speed based on remaining distance with the goal of coming to a perfect stop within
        // acceleration limits
        double perfectVelocity =
                Math.sqrt(2.0 * options.maxLinearAcceleration() * distanceToSegmentEnd);
        return Math.max(perfectVelocity, 0.05); // minimum of 2 in/s
    }

    private AutoConstraintCalculator() {}
}
