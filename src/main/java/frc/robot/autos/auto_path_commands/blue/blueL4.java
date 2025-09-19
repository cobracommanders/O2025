package frc.robot.autos.auto_path_commands.blue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;

public class blueL4 extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  public blueL4(RequestManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.redPose;
  }

  public Pose2d getStartingPosition(){
    return getStartingPose();
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.START_R1_AND_B1.bluePose),
                new AutoPoint(ReefPipe.PIPE_G.getPose(ReefPipeLevel.L4, RobotScoringSide.LEFT), 
                autoCommands.l4ApproachCommand(ReefPipe.PIPE_G, RobotScoringSide.LEFT))
                )));
  }
}