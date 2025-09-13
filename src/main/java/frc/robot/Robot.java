// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.commands.RobotCommands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.Lights.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
//import frc.robot.subsystems.led.LED;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Uncomment as needed
  public static RequestManager robotManager = RequestManager.getInstance();
  public static RobotCommands robotCommands = new RobotCommands();
  // public static final Controls controls = new Controls();
  private SendableChooser<Command> autoChooser;
  private final Timer seedImuTimer = new Timer();
  public static LED lights;
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

  }
  public final LEDPattern m_pattern = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  private final LEDPattern m_scrollingRainbow =
          m_pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    lights.periodic();
    MechanismVisualizer.publishData();
    FmsSubsystem.getInstance().updateSimulation();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotInit() {
    FmsSubsystem.getInstance();
    SmartDashboard.putData(autoChooser);
    lights = new LED();
    AutoAlign.getInstance();

  }


  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    seedImuTimer.reset();
    seedImuTimer.start();

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
