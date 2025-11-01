package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.PipeScoringLevel;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks.Lollipop;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public class FourCoralProcessor extends BaseAuto {
    public FourCoralProcessor(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                blocks.initialDriveToReefBackProcessor(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, PipeScoringLevel.L2).asProxy(),

                blocks.approachLollipop(Lollipop.RIGHT).withDeadline(
                        requestManager.prepareLollipopAndAwaitReady().asProxy()
                ),

                blocks.intakeLollipop(Lollipop.RIGHT).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_B, PipeScoringLevel.L4).asProxy(),

                blocks.approachLollipop(Lollipop.MIDDLE).withDeadline(
                        Commands.sequence(
                                requestManager.overrideArmAcceleration(6.0),
                                requestManager.prepareLollipopAndAwaitReady().asProxy(),
                                requestManager.clearOverrideArmAcceleration()
                        )
                ),

                blocks.intakeLollipop(Lollipop.MIDDLE).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, PipeScoringLevel.L4).asProxy(),

                blocks.approachLollipop(Lollipop.LEFT).withDeadline(
                        Commands.sequence(
                                requestManager.overrideArmAcceleration(6.0),
                                requestManager.prepareLollipopAndAwaitReady().asProxy(),
                                requestManager.clearOverrideArmAcceleration()
                        )
                ),

                blocks.intakeLollipop(Lollipop.LEFT).asProxy(),

                robotCommands.autoReefAlignAndScore(RobotScoringSide.LEFT, ReefPipe.PIPE_A, PipeScoringLevel.L2).asProxy()
        )
                .finallyDo(() -> {
                    requestManager.clearOverrideArmAcceleration().schedule();
                })
                ;
    }
}