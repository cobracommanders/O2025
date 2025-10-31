package frc.robot.autos.auto_path_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.PipeScoringLevel;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks.Lollipop;
import frc.robot.autos.BaseAuto;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public class OneCoralClimberSideBackReef extends BaseAuto {
    public OneCoralClimberSideBackReef(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                robotCommands.autoReefAlignAndScoreNearest(RobotScoringSide.LEFT, ReefPipe.PIPE_H, PipeScoringLevel.L4).asProxy(),
                requestManager.idleArm().asProxy()
        )
                .finallyDo(() -> {
                    requestManager.clearOverrideArmAcceleration().schedule();
                })
                ;
    }
}