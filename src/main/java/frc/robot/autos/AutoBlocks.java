package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.commands.RobotCommands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {

    private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE = new PoseErrorTolerance(0.6, 25);

    private static final PoseErrorTolerance LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.6, 10);

    private static final PoseErrorTolerance SUPER_FAST_LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.8, 20);
    public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);

    public static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET = new Transform2d(-0.6, -0.9, Rotation2d.kZero);

    private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET = new Transform2d(0, Units.inchesToMeters(-60),
            Rotation2d.kZero);

    private static final Transform2d CENTER_LOLLIPOP_OFFSET = new Transform2d(0, Units.inchesToMeters(5),
            Rotation2d.kZero);
    private static final Transform2d APPROACH_LOLLIPOP_OFFSET = new Transform2d(0, Units.inchesToMeters(15),
            Rotation2d.kZero);

    public static final Transform2d LOLLIPOP_OFFSET = new Transform2d(
            0.0,
            -Units.inchesToMeters(ArmConstants.inchesFromCenter),
            Rotation2d.fromDegrees(90));
    public static final AutoConstraintOptions MAX_CONSTRAINTS = new AutoConstraintOptions(4.75, 57, 4.0, 30);
    public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS = MAX_CONSTRAINTS.withMaxLinearVelocity(5)
            .withMaxLinearAcceleration(4.5);
    public static final AutoConstraintOptions BASE_CONSTRAINTS = new AutoConstraintOptions(4.0, 30, 2.5, 25);

    public static final AutoConstraintOptions CORAL_MAP_CONSTRAINTS = new AutoConstraintOptions(4.0, 10, 2.5, 10);
    private static final AutoConstraintOptions SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(1.5)
            .withMaxLinearAcceleration(1.75);
    private static final AutoConstraintOptions L2_SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(3.3)
            .withMaxLinearAcceleration(2.15);
    private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0)
            .withMaxLinearVelocity(1.7);

    private static final AutoConstraintOptions SUPER_FAST_LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS
            .withMaxLinearAcceleration(3.0).withMaxLinearVelocity(4.5);

    public static final AutoConstraintOptions BASE_CONSTRAINTS_FOR_GROUND_AUTOS = new AutoConstraintOptions(3.75, 57,
            1.75, 25);
    private static final AutoConstraintOptions SCORING_CONSTRAINTS_FOR_GROUND_AUTOS = BASE_CONSTRAINTS_FOR_GROUND_AUTOS
            .withMaxLinearAcceleration(1.25).withMaxLinearVelocity(3);

    private final Trailblazer trailblazer;

    private final AutoCommands autoCommands = AutoCommands.getInstance();

    public AutoBlocks(Trailblazer trailblazer) {
        this.trailblazer = trailblazer;
    }

    // public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe,
    // RobotScoringSide scoringSide) {
    // return Commands.sequence(
    // trailblazer.followSegment(
    // new AutoSegment(
    // BASE_CONSTRAINTS,
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
    // autoCommands.l4ApproachCommand(pipe, scoringSide),
    // BASE_CONSTRAINTS),
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
    // BASE_CONSTRAINTS),
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
    // autoCommands.l4ApproachCommand(pipe, scoringSide),
    // SCORING_CONSTRAINTS),
    // // Actually align to score
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
    // SCORING_CONSTRAINTS)
    // )
    // ),

    // trailblazer.followSegment(
    // new AutoSegment(
    // BASE_CONSTRAINTS,
    // AFTER_SCORE_POSITION_TOLERANCE,
    // // Start at the scoring position
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
    // Commands.waitSeconds(0.15)
    // .andThen(RobotCommands.getInstance().resetToIdleCommand())),
    // // Scoot back to the lineup position to finish the score
    // new AutoPoint(
    // () -> AutoAlign.getInstance().getUsedScoringPose(
    // pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT))
    // )
    // )
    // );
    // }

    public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                BASE_CONSTRAINTS,
                                AutoBlocks.APPROACH_REEF_TOLERANCE,
                                // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                                // RobotScoringSide.LEFT)),
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                        Robot.robotCommands.waitForAllIdle()
                                                .andThen(Robot.robotCommands.prepareScoreWithHandoffCheckCommand())))),
                trailblazer.followSegment(
                        new AutoSegment(
                                SCORING_CONSTRAINTS,
                                // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                                // RobotScoringSide.LEFT)),
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                        Robot.robotCommands.autoReefAlignCommand()
                                                .andThen(Robot.robotCommands.scoreCommand())))),
                ArmManager.getInstance().finishScoring(),
                // trailblazer.followSegment(
                // new AutoSegment(
                // SCORING_CONSTRAINTS,
                // AutoBlocks.APPROACH_REEF_TOLERANCE,
                // // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                // RobotScoringSide.LEFT)),
                // new AutoPoint(()-> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                // Robot.robotCommands.scoreCommand())
                // )
                // ),
                
                // trailblazer.followSegment(
                // new AutoSegment(
                // BASE_CONSTRAINTS,
                // // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                // RobotScoringSide.LEFT)),
                // new AutoPoint(()-> Points.START_R1_AND_B1.getPose())
                // )
                // ),
                trailblazer.followSegment(
                        new AutoSegment(
                                BASE_CONSTRAINTS,
                                AFTER_SCORE_POSITION_TOLERANCE,
                                // Start at the scoring position
                                new AutoPoint(
                                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                                pipe, ReefPipeLevel.L4, scoringSide),
                                        Commands.waitSeconds(0.15)
                                                .andThen(RobotCommands.getInstance().resetToIdleCommand())),
                                // Scoot back to the lineup position to finish the score
                                new AutoPoint(
                                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                                pipe, ReefPipeLevel.L4, scoringSide)))));
    }
    private Pose2d lollipopCoords = new Pose2d(FieldConstants.StagingPositions.iceCreams[1], Rotation2d.kZero);
    private Pose2d approachLolliOffset = new Pose2d(new Translation2d(1, 0), Rotation2d.kZero);
    private Pose2d getMidLolli() {
        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(lollipopCoords) : lollipopCoords;
    }
    private Pose2d getLolliApproach() {
        return getMidLolli().plus(approachLolliOffset.minus(Pose2d.kZero));
    }

    // private Pose2d approachLolli = new Pose2d(getMidLolli()., Rotation2d.fromDegrees());
    public Command pickUpMidLolli() {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                LOLLIPOP_CONSTRAINTS,
                                AutoBlocks.LOLLIPOP_APPROACH_TOLERANCE,
                                // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                                // RobotScoringSide.LEFT)),
                                new AutoPoint(()-> getLolliApproach(),
                                        Robot.robotCommands.setGroundAlgaeCommand().andThen(Robot.robotCommands.algaeIntakeCommand())
                                ),
                                new AutoPoint(()-> getMidLolli())
                        )
                )
        );
    }
    public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                BASE_CONSTRAINTS,
                                AutoBlocks.APPROACH_REEF_TOLERANCE,
                                // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                                // RobotScoringSide.LEFT)),
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                        Robot.robotCommands.waitForAllIdle()
                                                .andThen(Robot.robotCommands.prepareScoreWithHandoffCheckCommand())))),
                trailblazer.followSegment(
                        new AutoSegment(
                                SCORING_CONSTRAINTS,
                                // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4,
                                // RobotScoringSide.LEFT)),
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                        Robot.robotCommands.autoReefAlignCommand()
                                                .andThen(Robot.robotCommands.scoreCommand())))),
                ArmManager.getInstance().finishScoring(),
                trailblazer.followSegment(
                        new AutoSegment(
                                BASE_CONSTRAINTS,
                                AFTER_SCORE_POSITION_TOLERANCE,
                                // Start at the scoring position
                                new AutoPoint(
                                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                                pipe, ReefPipeLevel.L4, scoringSide),
                                        Commands.waitSeconds(0.15)
                                                .andThen(RobotCommands.getInstance().resetToIdleCommand())),
                                // Scoot back to the lineup position to finish the score
                                new AutoPoint(
                                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                                pipe, ReefPipeLevel.L4, scoringSide)))));
    }
}
