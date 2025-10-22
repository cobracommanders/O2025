package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.PipeScoringLevel;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks.Lollipop;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public class redFourCoralNonProcessor extends BaseAuto {
    public redFourCoralNonProcessor(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Pose2d getStartingPose() {
        return Points.START_R1_AND_B1.redPose;
    }

    double startTime = 0.0;

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                blocks.driveToBackReefRedNonProcessor(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, PipeScoringLevel.L4).asProxy(),

                Commands.parallel(
                        requestManager.prepareLollipopAndAwaitReady().asProxy(),
                        blocks.approachLollipop(Lollipop.LEFT)
                ),

                blocks.intakeLollipop(Lollipop.LEFT).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, PipeScoringLevel.L2).asProxy(),

                Commands.parallel(
                        requestManager.prepareLollipopAndAwaitReady().asProxy(),
                        blocks.approachLollipop(Lollipop.MIDDLE)
                ),

                blocks.intakeLollipop(Lollipop.MIDDLE).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, PipeScoringLevel.L2).asProxy(),

                Commands.parallel(
                        requestManager.prepareLollipopAndAwaitReady().asProxy(),
                        blocks.approachLollipop(Lollipop.RIGHT)
                ),

                blocks.intakeLollipop(Lollipop.RIGHT).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, PipeScoringLevel.L4).asProxy()
//                requestManager.idleAll().asProxy()
        )
                .beforeStarting(() -> startTime = Timer.getTimestamp())
                .finallyDo(() -> System.out.println("Total time: " + (Timer.getTimestamp() - startTime)))
                ;
    }
}