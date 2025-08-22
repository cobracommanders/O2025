// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static RobotManager robotManager = RobotManager.getInstance();
  public static RobotCommands robotCommands = new RobotCommands();
  // public static OperatorOptions operatorOptions = OperatorOptions.getInstance();

  public Robot() {
    
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void robotInit(){
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

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
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // arm.setState(ArmStates.IDLE);
    // elevator.setState(ElevatorStates.L4);
  }

  @Override
  public void testExit() {}
}
