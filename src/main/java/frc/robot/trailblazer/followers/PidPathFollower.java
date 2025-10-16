package frc.robot.trailblazer.followers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PolarChassisSpeeds;

public class PidPathFollower implements PathFollower {
    private final PIDController linearController;
    private final PIDController thetaController;

    private static final double ROTATION_FEED_FORWARD = 0.01;

    public PidPathFollower(PIDController linearController, PIDController thetaController) {
        this.linearController = linearController;
        this.thetaController = thetaController;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(0.01);
    }

    @Override
    public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d targetPose) {
        double rotationSpeed =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

        if (!MathUtil.isNear(
                targetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                1.0)) {
            rotationSpeed +=
                    Math.copySign(Units.rotationsToRadians(ROTATION_FEED_FORWARD), rotationSpeed);
        }

        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(xError, yError);
        Rotation2d direction = new Rotation2d(xError, yError);

        return new PolarChassisSpeeds(
                -linearController.calculate(distance, 0.0), direction, rotationSpeed);
    }
}
