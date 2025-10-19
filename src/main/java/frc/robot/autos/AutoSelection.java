package frc.robot.autos;
import java.util.function.BiFunction;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import frc.robot.autos.auto_path_commands.blue.blueL4;
import frc.robot.autos.auto_path_commands.red.redL4;
import frc.robot.commands.RobotCommands;
import frc.robot.autos.auto_path_commands.blue.blueFourCoralNonProcessor;
import frc.robot.autos.auto_path_commands.blue.blueFourCoralProcessor;
import frc.robot.autos.auto_path_commands.red.redFourCoralNonProcessor;
import frc.robot.autos.auto_path_commands.red.redFourCoralProcessor;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public enum AutoSelection implements AutoSelectionBase {
  L4(redL4::new, blueL4::new),
  FOUR_CORAL_NON_PROCESSOR(redFourCoralNonProcessor::new, blueFourCoralNonProcessor::new),
  FOUR_CORAL_PROCESSOR(redFourCoralProcessor::new, blueFourCoralProcessor::new);

  public final TriFunction<RequestManager, Trailblazer,RobotCommands, BaseAuto> redAuto;
  public final TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> blueAuto;

  private AutoSelection(
      TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> redAuto,
      TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}