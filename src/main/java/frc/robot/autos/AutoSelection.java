package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import frc.robot.commands.RobotCommands;
import frc.robot.autos.auto_path_commands.FourCoralNonProcessor;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public enum AutoSelection implements AutoSelectionBase {
  // L4(redL4::new, blueL4::new),
  FOUR_CORAL_NON_PROCESSOR(FourCoralNonProcessor::new, FourCoralNonProcessor::new);//,
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