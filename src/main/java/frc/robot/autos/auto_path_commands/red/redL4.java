package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.autos.AutoBlocks.Lollipop;
import frc.robot.commands.RobotCommands;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.RequestManagerStates;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpers;

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
      blocks.scorePreloadL4AutoAlignFirst(ReefPipe.PIPE_J, RobotScoringSide.LEFT),
      blocks.backUpFromReef(ReefPipe.PIPE_J, RobotScoringSide.LEFT),
      RobotCommands.getInstance().autoLollipopIntakeCommand(),
      // Commands.parallel(
        
      blocks.pickUpLolli(Lollipop.LEFT, ReefPipe.PIPE_J, RobotScoringSide.LEFT),
      // ),
      blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
      RobotCommands.getInstance().autoLollipopIntakeCommand(),

      // RobotCommands.getInstance().waitForAllIdle(),
      // RobotCommands.getInstance().lollipopIntakeCommand(),
      blocks.pickUpLolli(Lollipop.MIDDLE, ReefPipe.PIPE_A, RobotScoringSide.LEFT),
      blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
      RobotCommands.getInstance().autoLollipopIntakeCommand(),

      // RobotCommands.getInstance().waitForAllIdle(),
      // RobotCommands.getInstance().lollipopIntakeCommand(),
      blocks.pickUpLolli(Lollipop.RIGHT, ReefPipe.PIPE_B, RobotScoringSide.LEFT),
      blocks.scoreL4(ReefPipe.PIPE_C, RobotScoringSide.LEFT),
      blocks.backUpFromReef(ReefPipe.PIPE_C, RobotScoringSide.LEFT)

    );
  }
}