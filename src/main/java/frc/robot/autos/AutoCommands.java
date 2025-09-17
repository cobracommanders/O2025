package frc.robot.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.RequestManagerStates;

public class AutoCommands {
  private final RobotCommands robotCommands = RobotCommands.getInstance();
  private final RequestManager robotManager = RequestManager.getInstance();
  private final AutoAlign autoAlign = AutoAlign.getInstance();

  public AutoCommands() {
  }

  public boolean alignedForScore() {
    return autoAlign.isAligned();
  }

//   public Command intakeLollipopCommand() {
//     return Commands.runOnce(robotManager::lollipopIntakeGrabRequest)
//         .withName("LollipopIntakeCommand");
//   }

  public Command l4ApproachCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              autoAlign.setAutoReefPipeOverride(pipe);
                robotCommands.setL4Command().andThen(robotCommands.prepareScoreCommand());
            })
        .withName("L4ApproachCommand");
  }

  public Command l4LeftReleaseCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              autoAlign.setAutoReefPipeOverride(pipe);
              robotCommands.scoreCommand();
            })
        .withName("L4LeftReleaseCommand");
  }

  public Command waitForAlignedForScore() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return Commands.waitUntil(this::alignedForScore)
        .withTimeout(5)
        .withName("WaitForAlignedForScore");
  }

  public Command groundIntakeToL4Command() {
    return Commands.runOnce(
            () -> {
              robotManager.handoffRequest();
              robotCommands.setL4Command().andThen(robotCommands.prepareScoreCommand());
            })
        .withName("GroundIntakeL4Command");
  }

  private static AutoCommands instance;

  public static AutoCommands getInstance() {
      if (instance == null)
          instance = new AutoCommands(); // Make sure there is an instance (this will only run once)
      return instance;
  }
  
}