package frc.robot.autoAlign.tagAlign;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.ReefSide;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpersDog;
import frc.robot.util.PolarChassisSpeeds;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class TagAlign {
    public static final List<ReefPipe> ALL_REEF_PIPES =
            List.copyOf(Arrays.asList(ReefPipe.values()));
    public static final List<ReefSide> ALL_REEF_SIDES =
            List.copyOf(Arrays.asList(ReefSide.values()));

    private static final ProfiledPIDController REEF_PIPE_ROTATION_CONTROLLER =
            new ProfiledPIDController(
                    5.75,
                    0.0,
                    0.0,
                    new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));
    private static final ProfiledPIDController ALGAE_ROTATION_CONTROLLER =
            new ProfiledPIDController(
                    5.75,
                    0.0,
                    0.0,
                    new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));
    private static final ProfiledPIDController L1_ROTATION_CONTROLLER =
            new ProfiledPIDController(
                    5.75,
                    0.0,
                    0.0,
                    new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

    private static final ProfiledPIDController REEF_PIPE_TRANSLATION_CONTROLLER =
            new ProfiledPIDController(
                    4.0,
                    0.0,
                    0.0,
                    new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

    private static final ProfiledPIDController ALGAE_TRANSLATION_CONTROLLER =
            new ProfiledPIDController(
                    4.0,
                    0.0,
                    0.0,
                    new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

    private static final ProfiledPIDController L1_TRANSLATION_CONTROLLER =
            new ProfiledPIDController(4.0, 0.0, 0.0, new Constraints(3.0, 2.0));

    private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
            DogLog.tunable("AutoAlign/IsAlignedTranslation", Units.inchesToMeters(1));
    private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
            DogLog.tunable("AutoAlign/IsAlignedRotation", 1.0);

    private static final DoubleSubscriber IN_RANGE_TRANSLATION_THRESHOLD =
            DogLog.tunable("AutoAlign/InRangeTranslation", Units.inchesToMeters(1.5));
    private static final DoubleSubscriber IN_RANGE_ROTATION_THRESHOLD =
            DogLog.tunable("AutoAlign/InRangeRotation", 1.0);

    private static final double IDEAL_L1_OFFSET = Units.inchesToMeters(0.5);
    private static final Transform2d LEFT_L1_TRANSFORM =
            new Transform2d(0, IDEAL_L1_OFFSET, Rotation2d.fromDegrees(0));
    private static final Transform2d RIGHT_L1_TRANSFORM =
            new Transform2d(0, -IDEAL_L1_OFFSET, Rotation2d.fromDegrees(0));

    private final AlignmentCostUtil alignmentCostUtil;
    private final LocalizationSubsystem localization;

    private ReefPipeLevel pipeLevel = ReefPipeLevel.RAISING;
    private ReefPipeLevel preferredScoringLevel = ReefPipeLevel.L4;
    private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
    private boolean aligned = false;
    private boolean translationGood = false;
    private boolean resetReefPipeNextLoop = false;
    private boolean resetAlgaeNextLoop = false;
    private boolean resetL1NextLoop = false;

    private static final DoubleSubscriber FEED_FORWARD = DogLog.tunable("AutoAlign/FeedForward", 0.0);


    public TagAlign(DriveSubsystem swerve, LocalizationSubsystem localization) {
        this.localization = localization;
        alignmentCostUtil = new AlignmentCostUtil(localization, swerve, robotScoringSide);
        ALGAE_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        L1_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        REEF_PIPE_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setLevel(ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
        this.robotScoringSide = side;
        alignmentCostUtil.setSide(robotScoringSide);
        this.pipeLevel = level;
        this.preferredScoringLevel = preferredLevel;
    }

    public boolean isAligned(ReefPipe pipe) {
        if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
            return false;
        }
        var robotPose = localization.getPose();
        var targetPose = getUsedScoringPose(pipe);

        translationGood =
                (robotPose.getTranslation().getDistance(targetPose.getTranslation())
                        <= TRANSLATION_GOOD_THRESHOLD.get());
        boolean rotationGood = MathUtil.isNear(
                targetPose.getRotation().getDegrees(),
                robotPose.getRotation().getDegrees(),
                ROTATION_GOOD_THRESHOLD.get(),
                -180.0,
                180.0);

        DogLog.log("AutoAlign/TranslationGood", translationGood);
        DogLog.log("AutoAlign/RotationGood", rotationGood);

        return translationGood && rotationGood;
    }

    public boolean needToMove(Pose2d goal) {
        return !inRange(goal);
    }

    public boolean inRange(Pose2d goal) {
        var robotPose = localization.getPose();
        var translationGood =
                (robotPose.getTranslation().getDistance(goal.getTranslation())
                        <= IN_RANGE_TRANSLATION_THRESHOLD.get());
        var rotationGood =
                MathUtil.isNear(
                        goal.getRotation().getDegrees(),
                        robotPose.getRotation().getDegrees(),
                        IN_RANGE_ROTATION_THRESHOLD.get(),
                        -180.0,
                        180.0);
        return translationGood && rotationGood;
    }

    public void reset() {
        resetL1NextLoop = true;
        resetAlgaeNextLoop = true;
        resetReefPipeNextLoop = true;
    }

    public Pose2d getUsedScoringPose(ReefPipe pipe) {
        return getUsedScoringPose(pipe, robotScoringSide);
    }

    public Pose2d getUsedScoringPose(ReefPipe pipe, RobotScoringSide side) {
        if (OperatorOptions.getInstance().isCoralScoringL1()) {
            var reefPose = ReefSide.fromPipe(pipe).getPose(localization.getPose());
            var pipePose = pipe.getPose(pipeLevel, side, localization.getPose());
            var rotatedPipePose =
                    pipePose.rotateAround(
                            reefPose.getTranslation(),
                            Rotation2d.fromDegrees(360 - reefPose.getRotation().getDegrees()));
            var isLeft = rotatedPipePose.getY() > reefPose.getY();
            if (isLeft) {
                return pipePose.transformBy(LEFT_L1_TRANSFORM);
            }
            return pipePose.transformBy(RIGHT_L1_TRANSFORM);
        }
        return pipe.getPose(pipeLevel, side, localization.getPose());
    }

    /**
     * Returns the best reef pipe for scoring, based on the robot's current state.
     */
    public ReefPipe getBestPipe() {
        var level = pipeLevel;
        var robotPose = localization.getPose();
        if (pipeLevel.equals(ReefPipeLevel.L1)) {
            return getClosestPipe();
        }
        if (pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
            return ALL_REEF_PIPES.stream()
                    .min(
                            Comparator.comparingDouble(
                                    pipe ->
                                            robotPose
                                                    .getTranslation()
                                                    .getDistance(
                                                            pipe.getPose(ReefPipeLevel.BACK_AWAY, robotScoringSide, robotPose)
                                                                    .getTranslation())))
                    .orElseThrow();
        }
        if (pipeLevel.equals(ReefPipeLevel.RAISING)) {
            level = preferredScoringLevel;
        }
        return ALL_REEF_PIPES.stream()
                .min(alignmentCostUtil.getReefPipeComparator(level))
                .orElseThrow();
    }

    public ReefPipe getClosestPipe() {
        var robotPose = localization.getPose();
        return ALL_REEF_PIPES.stream()
                .min(
                        Comparator.comparingDouble(
                                pipe ->
                                        robotPose
                                                .getTranslation()
                                                .getDistance(
                                                        pipe.getPose(pipeLevel, robotScoringSide, robotPose).getTranslation())))
                .orElseThrow();
    }

    public ReefSide getBestAlgaeSide() {
        return ALL_REEF_SIDES.stream().min(alignmentCostUtil.getAlgaeComparator()).orElseThrow();
    }

    public PolarChassisSpeeds getReefPipeAlignmentChassisSpeeds(
            Pose2d targetPose,
            Pose2d currentPose,
            AutoConstraintOptions constraints,
            PolarChassisSpeeds currentSpeeds) {
        var reset = resetReefPipeNextLoop;
        resetReefPipeNextLoop = false;
        return getPoseAlignmentChassisSpeeds(
                targetPose,
                currentPose,
                REEF_PIPE_TRANSLATION_CONTROLLER,
                REEF_PIPE_ROTATION_CONTROLLER,
                constraints,
                currentSpeeds,
                reset);
    }

    public PolarChassisSpeeds getAlgaeAlignmentChassisSpeeds(
            Pose2d targetPose,
            Pose2d currentPose,
            AutoConstraintOptions constraints,
            PolarChassisSpeeds currentSpeeds) {
        var reset = resetAlgaeNextLoop;
        resetAlgaeNextLoop = false;
        return getPoseAlignmentChassisSpeeds(
                targetPose,
                currentPose,
                ALGAE_TRANSLATION_CONTROLLER,
                ALGAE_ROTATION_CONTROLLER,
                constraints,
                currentSpeeds,
                reset);
    }

    public PolarChassisSpeeds getL1AlignmentChassisSpeeds(
            Pose2d targetPose,
            Pose2d currentPose,
            AutoConstraintOptions constraints,
            PolarChassisSpeeds currentSpeeds) {
        var reset = resetL1NextLoop;
        resetL1NextLoop = false;
        return getPoseAlignmentChassisSpeeds(
                targetPose,
                currentPose,
                L1_TRANSLATION_CONTROLLER,
                L1_ROTATION_CONTROLLER,
                constraints,
                currentSpeeds,
                reset);
    }

    private PolarChassisSpeeds getPoseAlignmentChassisSpeeds(
            Pose2d targetPose,
            Pose2d currentPose,
            ProfiledPIDController translationController,
            ProfiledPIDController rotationController,
            AutoConstraintOptions constraints,
            PolarChassisSpeeds currentSpeeds,
            boolean reset) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        if (FeatureFlags.AUTO_ALIGN_DEADBAND.getAsBoolean()) {
            if (aligned || inRange(targetPose)) {
                if (needToMove(targetPose)) {
                    aligned = false;
                } else {
                    aligned = true;
                    return new PolarChassisSpeeds();
                }
            }
        }

        // Calculate x and y velocities
        double distanceToGoalMeters =
                currentPose.getTranslation().getDistance(targetPose.getTranslation());

        if (reset) {
            DogLog.timestamp("AutoAlign/ResetControllers");
            rotationController.reset(
                    currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
            rotationController.setGoal(targetPose.getRotation().getRadians());
            translationController.reset(
                    distanceToGoalMeters,
                    Math.min(
                            0.0,
                            -new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                                    .rotateBy(
                                            targetPose
                                                    .getTranslation()
                                                    .minus(currentPose.getTranslation())
                                                    .getAngle()
                                                    .unaryMinus())
                                    .getX()));
            translationController.setGoal(0);
        }

        var driveVelocityMagnitude =
                translationController.calculate(
                        distanceToGoalMeters,
                        new State(0, 0),
                        new Constraints(constraints.maxLinearVelocity(), constraints.maxLinearAcceleration()));

        if (!translationGood) {
            driveVelocityMagnitude += Math.copySign(FEED_FORWARD.get(), driveVelocityMagnitude);
        }

        DogLog.log("AutoAlign/setpoint", Units.radiansToDegrees(distanceToGoalMeters));

        var rotationSpeed =
                rotationController.calculate(
                        currentPose.getRotation().getRadians(),
                        new State(targetPose.getRotation().getRadians(), 0),
                        constraints.getAngularConstraints());

        var driveDirection = MathHelpersDog.getDriveDirection(currentPose, targetPose);

        DogLog.log("AutoAlign/DistanceToGoal", distanceToGoalMeters);
        DogLog.log("AutoAlign/DriveVelocityMagnitude", driveVelocityMagnitude);
        DogLog.log("AutoAlign/RotationSpeed", rotationSpeed);
        DogLog.log("AutoAlign/DriveDirection", driveDirection.getDegrees());

        return new PolarChassisSpeeds(driveVelocityMagnitude, driveDirection, rotationSpeed);
    }
}