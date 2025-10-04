package frc.robot.autoAlign.tagAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autoAlign.*;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.util.MathHelpersDog;

import java.util.Comparator;

public class AlignmentCostUtil {
    private static final double DRIVE_DIRECTION_SCALAR = 0.02;
    private static final double ANGLE_ERROR_SCALAR = 0.3;

    private static final double DRIVE_DIRECTION_SCALAR_CORAL = 0.02;

    /**
     * Returns the "cost" (a dimensionless number) of aligning to a given pose based on the robot's
     * current state.
     *
     * @param target        Pose to align to
     * @param robotPose     The robot's current pose
     * @param robotVelocity The robot's current velocity (field relative)
     */
    public static double getAlignCost(Pose2d target, Pose2d robotPose, ChassisSpeeds robotVelocity) {
        var distanceCost =
                FeatureFlags.AUTO_ALIGN_DISTANCE_COST.getAsBoolean()
                        ? target.getTranslation().getDistance(robotPose.getTranslation())
                        : 0.0;
        if (target.equals(Pose2d.kZero) || robotPose.equals(Pose2d.kZero)) {
            return distanceCost;
        }

        var angleToAim = target.getRotation().getRadians();

        var angleError =
                Math.abs(MathUtil.angleModulus(angleToAim - robotPose.getRotation().getRadians()));

        var angleErrorCost =
                FeatureFlags.AUTO_ALIGN_HEADING_COST.getAsBoolean() ? angleError * ANGLE_ERROR_SCALAR : 0.0;

        if (Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) == 0.0) {
            return distanceCost + angleErrorCost;
        }

        var targetRobotRelative = target.minus(robotPose);
        var targetDirection = targetRobotRelative.getTranslation().getAngle();

        var driveAngleCost =
                FeatureFlags.AUTO_ALIGN_DRIVE_DIRECTION_COST.getAsBoolean()
                        ? DRIVE_DIRECTION_SCALAR
                        * Math.abs(
                        targetDirection.minus(MathHelpersDog.vectorDirection(robotVelocity)).getRadians())
                        : 0.0;
        return distanceCost + angleErrorCost + driveAngleCost;
    }

    private final LocalizationSubsystem localization;
    private final DriveSubsystem swerve;
    private RobotScoringSide side;

    private final Comparator<ReefPipe> pipeL4Comparator = createReefPipeComparator(ReefPipeLevel.L4);
    private final Comparator<ReefPipe> pipeL3Comparator = createReefPipeComparator(ReefPipeLevel.L3);
    private final Comparator<ReefPipe> pipeL2Comparator = createReefPipeComparator(ReefPipeLevel.L2);
    private final Comparator<ReefPipe> pipeL1Comparator = createReefPipeComparator(ReefPipeLevel.L1);
    private final Comparator<ReefSide> algaeComparator = createAlgaeComparator();

    public AlignmentCostUtil(
            LocalizationSubsystem localization,
            DriveSubsystem swerve,
            RobotScoringSide side) {
        this.localization = localization;
        this.swerve = swerve;
        this.side = side;
    }

    public Comparator<ReefPipe> getReefPipeComparator(ReefPipeLevel level) {
        return switch (level) {
            case L4 -> pipeL4Comparator;
            case L3 -> pipeL3Comparator;
            case L2 -> pipeL2Comparator;
            case L1 -> pipeL1Comparator;
            // Shouldn't ever happen
            default -> pipeL2Comparator;
        };
    }

    public Comparator<ReefSide> getAlgaeComparator() {
        return algaeComparator;
    }

    public void setSide(RobotScoringSide side) {
        this.side = side;
    }

    /**
     * Helper function to create a singleton comparator for each level.
     */
    private Comparator<ReefPipe> createReefPipeComparator(ReefPipeLevel level) {

        return switch (level) {
            case L1 -> {
                yield Comparator.comparingDouble(
                        pipe -> {
                            var allPipes =
                                    TagAlign.ALL_REEF_PIPES; // Assuming reefState has a method to get all pipes
                            return allPipes.stream()
                                    .filter(p -> p.getPose(level, side, localization.getPose()) != null)
                                    .min(
                                            Comparator.comparingDouble(
                                                    p ->
                                                            p.getPose(level, side, localization.getPose())
                                                                    .getTranslation()
                                                                    .getDistance(localization.getPose().getTranslation())))
                                    .map(
                                            p ->
                                                    getAlignCost(
                                                            p.getPose(level, side, localization.getPose()),
                                                            localization.getPose(),
                                                            swerve.getTeleopSpeeds()))
                                    .orElse(Double.MAX_VALUE);
                        });
            }
            default -> {
                yield Comparator.comparingDouble(
                        pipe ->
                                getAlignCost(
                                        pipe.getPose(level, side, localization.getPose()),
                                        localization.getPose(),
                                        swerve.getTeleopSpeeds()));
            }
        };
    }

    private Comparator<ReefSide> createAlgaeComparator() {
        return Comparator.comparingDouble(
                side ->
                        getAlignCost(
                                side.getPose(localization.getPose()),
                                localization.getPose(),
                                swerve.getTeleopSpeeds()));
    }
}