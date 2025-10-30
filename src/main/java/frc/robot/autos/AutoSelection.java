package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import frc.robot.autos.auto_path_commands.OneCoralClimberSideBackReef;
import frc.robot.commands.RobotCommands;
import frc.robot.autos.auto_path_commands.FourCoralNonProcessor;
import frc.robot.autos.auto_path_commands.FourCoralProcessor;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public enum AutoSelection implements AutoSelectionBase {
  FOUR_CORAL_NON_PROCESSOR(FourCoralNonProcessor::new),
  FOUR_CORAL_PROCESSOR(FourCoralProcessor::new),
  ONE_CORAL_CLIMBERSIDE_POLE_H(OneCoralClimberSideBackReef::new);

  public final TriFunction<RequestManager, Trailblazer,RobotCommands, BaseAuto> auto;

  AutoSelection(TriFunction<RequestManager, Trailblazer, RobotCommands, BaseAuto> auto) {
    this.auto = auto;
  }
}