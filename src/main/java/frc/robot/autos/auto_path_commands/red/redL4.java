package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.AutoCommands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.commands.RobotCommands;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;

public class redL4 extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  public redL4(RequestManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
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
      Commands.parallel(
        trailblazer.followSegment(
          new AutoSegment(
              CONSTRAINTS,
              AutoBlocks.APPROACH_REEF_TOLERANCE,
              new AutoPoint(Points.START_R1_AND_B1.redPose),
              // new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT)),
              new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT),
                  Robot.robotCommands.prepareScoreWithHandoffCheckCommand()
          )))
      ).withDeadline(Robot.robotCommands.waitForL4()), Robot.robotCommands.autoReefAlignCommand()
        );
  }
}