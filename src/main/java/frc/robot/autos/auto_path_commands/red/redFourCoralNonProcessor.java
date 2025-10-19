package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.trailblazer.constraints.AutoConstraintOptions;

public class redFourCoralNonProcessor extends BaseAuto {
    public redFourCoralNonProcessor(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Pose2d getStartingPose() {
        return Points.START_R1_AND_B1.redPose;
    }

    public Pose2d getStartingPosition() {
        return getStartingPose();
    }

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                blocks.driveToBackReefRedNonProcessor(),
                
                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, ReefPipeLevel.L4, PipeScoringLevel.L4),

                Commands.parallel(
                    requestManager.prepareLollipopAndAwaitReady(),
                    blocks.pickUpLolli(Lollipop.RIGHT)
                ),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, ReefPipeLevel.L2, PipeScoringLevel.L2),

                Commands.parallel(
                    requestManager.prepareLollipopAndAwaitReady(),
                    blocks.pickUpLolli(Lollipop.MIDDLE)
                ),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, ReefPipeLevel.L2, PipeScoringLevel.L2),

                Commands.parallel(
                    requestManager.prepareLollipopAndAwaitReady(),
                    blocks.pickUpLolli(Lollipop.LEFT)
                ),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, ReefPipeLevel.L4, PipeScoringLevel.L4)
        );
    }
}