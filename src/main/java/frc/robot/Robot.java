// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.MechanismVisualizer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RobotManager;
import frc.robot.subsystems.MechanismVisualizer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Uncomment as needed
  public static RequestManager robotManager = RequestManager.getInstance();
  public static RobotCommands robotCommands = new RobotCommands();
  public static final Controls controls = new Controls();
  private SendableChooser<Command> autoChooser;

  public Robot() {
    for (Command command : robotCommands.getPathplannerCommands()) {
      NamedCommands.registerCommand(command.getName(), command);
    }
    Command centerL1 = AutoBuilder.buildAuto("CenterL1");
    Command centerL4 = AutoBuilder.buildAuto("CenterL4");

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("CenterL1", centerL1);
    autoChooser.addOption("CenterL4", centerL4);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    MechanismVisualizer.publishData();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotInit() {
    SmartDashboard.putData(autoChooser);
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    if (autoChooser.getSelected() != null)
      autoChooser.getSelected().schedule();
    DogLog.log("Selected Auto", autoChooser.getSelected().getName());
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Controls.getInstance().configureDriverCommands();
    Controls.getInstance().configureOperatorCommands();
    Controls.getInstance().configureDefaultCommands();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    Climber.getInstance().setWinchSpeed(-WinchSpeeds.DEPLOYING);
    // arm.setState(ArmStates.IDLE);
    // elevator.setState(ElevatorStates.L4);
  }

  @Override
  public void testExit() {
    Climber.getInstance().setWinchSpeed(0);
  }
}
