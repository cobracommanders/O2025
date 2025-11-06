package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.PipeScoringLevel;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseAuto;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseErrorTolerance;

public class SuperSpecialAuto extends BaseAuto {
    public SuperSpecialAuto(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        super(robotManager, trailblazer, robotCommands);
    }

    @Override
    protected Command createAutoCommand() {
        return Commands.sequence(
                robotCommands.autoReefAlignAndScoreNearest(RobotScoringSide.LEFT, ReefPipe.PIPE_E, PipeScoringLevel.L4).asProxy(),
                trailblazer.followSegment(new AutoSegment(
                        AutoBlocks.maximumConstraints.withMaxLinearAcceleration(3.0),
                        new PoseErrorTolerance(Units.inchesToMeters(5.0), 2.0),
                        new AutoPoint(() -> {
                            Pose2d waypoint =  new Pose2d(2.45, 1.15, Rotation2d.fromDegrees(- 45 - 90));
                            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? waypoint : MathHelpers.pathflip(waypoint);
                        })
                )).asProxy().alongWith(
                        Commands.sequence(
                                requestManager.idleAll().asProxy(),
                                requestManager.coralIntakeUntilPiece().asProxy()
                        )
                ).asProxy(),
                robotCommands.handoffAutoReefAlignAndScoreNearest(RobotScoringSide.LEFT, ReefPipe.PIPE_D, PipeScoringLevel.L4).asProxy(),
                Commands.waitSeconds(0.25),
                trailblazer.followSegment(new AutoSegment(
                        AutoBlocks.maximumConstraints.withMaxLinearAcceleration(3.0),
                        new PoseErrorTolerance(Units.inchesToMeters(5.0), 2.0),
                        new AutoPoint(() -> {
                            Pose2d waypoint =  new Pose2d(2.45, 1.15, Rotation2d.fromDegrees(- 45 - 90));
                            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? waypoint : MathHelpers.pathflip(waypoint);
                        })
                )).asProxy().alongWith(
                        Commands.sequence(
                                requestManager.idleAll().asProxy(),
                                requestManager.coralIntakeUntilPiece().asProxy()
                        )
                ).asProxy(),
                robotCommands.handoffAutoReefAlignAndScoreNearest(RobotScoringSide.LEFT, ReefPipe.PIPE_C, PipeScoringLevel.L4).asProxy()
        )
                .finallyDo(() -> {
                    requestManager.clearOverrideArmAcceleration().schedule();
                })
                ;
    }
}