package frc.robot.autos;
import java.util.function.BiFunction;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RobotCommands;
import frc.robot.autos.auto_path_commands.red.redFourCoralNonProcessor;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public enum AutoSelection implements AutoSelectionBase {
  // L4(redL4::new, blueL4::new),
  FOUR_CORAL_NON_PROCESSOR(redFourCoralNonProcessor::new, redFourCoralNonProcessor::new);//,
  // FOUR_CORAL_PROCESSOR(redFourCoralProcessor::new, blueFourCoralProcessor::new);

  public final TriFunction<RequestManager, Trailblazer,RobotCommands, BaseAuto> redAuto;
  public final TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> blueAuto;

  private AutoSelection(
      TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> redAuto,
      TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}