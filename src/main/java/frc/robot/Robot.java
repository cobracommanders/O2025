// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static OperatorOptions operatorOptions = OperatorOptions.getInstance();

  // Uncomment as needed
  public static RequestManager robotManager = RequestManager.getInstance();
  public static RobotCommands robotCommands = new RobotCommands();
  public static final Controls controls = new Controls();
  private SendableChooser<Command> autoChooser;    //LED.getInstance();
  // public static OperatorOptions operatorOptions =
  // OperatorOptions.getInstance();

  public Robot() {
    for (Command command : robotCommands.getPathplannerCommands()) {
      NamedCommands.registerCommand(command.getName(), command);
    }
    Command centerL1 = AutoBuilder.buildAuto("CenterL1");
    Command centerL4 = AutoBuilder.buildAuto("CenterL4");

    autoChooser = new SendableChooser<Command>();
    // autoChooser.addOption("CenterL1", centerL1);
    autoChooser.setDefaultOption("CenterL1", centerL1);
    autoChooser.addOption("CenterL4", centerL4);
    //LED Initilization

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    LED led = new LED(operatorOptions);


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
