package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks.Lollipop;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;

public class blueFourCoralNonProcessor extends BaseAuto {
    private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 1, 30);

    public blueFourCoralNonProcessor(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Pose2d getStartingPose() {
        return Points.START_R1_AND_B1.bluePose;
    }

    public Pose2d getStartingPosition() {
        return getStartingPose();
    }

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                blocks.driveToBackReefBlueNonProcessor(),
                blocks.scorePreloadL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                blocks.backUpFromReef(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                requestManager.prepareLollipopAndAwaitReady(),
                // Commands.parallel(

                blocks.pickUpLolli(Lollipop.RIGHT, ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                // ),
                blocks.scoreL2(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                blocks.backUpFromReef(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                requestManager.prepareLollipopAndAwaitReady(),

                // RobotCommands.getInstance().waitForAllIdle(),
                // RobotCommands.getInstance().lollipopIntakeCommand(),
                blocks.pickUpLolli(Lollipop.MIDDLE, ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                blocks.scoreL2(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                blocks.backUpFromReef(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                requestManager.prepareLollipopAndAwaitReady(),

                // RobotCommands.getInstance().waitForAllIdle(),
                // RobotCommands.getInstance().lollipopIntakeCommand(),
                blocks.pickUpLolli(Lollipop.LEFT, ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                blocks.backUpFromReef(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
                blocks.backUpFromReef(ReefPipe.PIPE_B, RobotScoringSide.LEFT)

        );
    }
}