package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {

    public static final AutoConstraintOptions BASE_CONSTRAINTS =
            new AutoConstraintOptions(4.0, 30, 2.5, 25);

    private static final AutoConstraintOptions SCORING_CONSTRAINTS =
            BASE_CONSTRAINTS.withMaxLinearVelocity(1.5)
                            .withMaxLinearAcceleration(1.75);

    private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
            new PoseErrorTolerance(0.6, 25);
            
    public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);

    private final Trailblazer trailblazer;

    private final AutoCommands autoCommands = AutoCommands.getInstance();

    public AutoBlocks(Trailblazer trailblazer) {
        this.trailblazer = trailblazer;
    }

    public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
            trailblazer.followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                        autoCommands.l4ApproachCommand(pipe, scoringSide),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                        autoCommands.l4ApproachCommand(pipe, scoringSide),
                        SCORING_CONSTRAINTS),
                    // Actually align to score
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                        SCORING_CONSTRAINTS)
                )
            ),

            trailblazer.followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    AFTER_SCORE_POSITION_TOLERANCE,
                    // Start at the scoring position
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                        Commands.waitSeconds(0.15)
                            .andThen(RobotCommands.getInstance().resetToIdleCommand())),
                    // Scoot back to the lineup position to finish the score
                    new AutoPoint(
                        () -> AutoAlign.getInstance().getUsedScoringPose(
                                pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT))
                )
            )
        );
    }

    public Command scorePreloadL3(Pose2d startingPose, ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
            trailblazer.followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    AutoBlocks.APPROACH_REEF_TOLERANCE,
                    // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT)),
                    new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                        Robot.robotCommands.waitForAllIdle().andThen(Robot.robotCommands.prepareScoreWithHandoffCheckCommand())
                    )
                )
            ),
            trailblazer.followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT)),
                    new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                        Robot.robotCommands.autoReefAlignCommand().andThen(Robot.robotCommands.scoreCommand()))
                )
            ),
            // trailblazer.followSegment(
            //     new AutoSegment(
            //         SCORING_CONSTRAINTS,
            //         AutoBlocks.APPROACH_REEF_TOLERANCE,
            //         // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT)),
            //         new AutoPoint(()-> pipe.getPose(ReefPipeLevel.L4, scoringSide),
            //             Robot.robotCommands.scoreCommand())
            //     )
            // ),
            Commands.waitSeconds(2),
            trailblazer.followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT)),
                    new AutoPoint(()-> Points.START_R1_AND_B1.getPose())
                )
            )
        );
    }
}
