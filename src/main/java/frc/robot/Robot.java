// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autos.AutoSelection;
import frc.robot.autos.Autos;
import frc.robot.commands.RobotCommands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.Lights.LED;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.LocalizationBase;
import frc.robot.trailblazer.SwerveBase;
//import frc.robot.subsystems.led.LED;
import frc.robot.trailblazer.Trailblazer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Command autonomousCommand = Commands.none();

  // Uncomment as needed
  public static RequestManager robotManager = RequestManager.getInstance();
  public static RobotCommands robotCommands = RobotCommands.getInstance();
  public static DriveSubsystem swerve = DriveSubsystem.getInstance();
  public static LocalizationSubsystem localization = LocalizationSubsystem.getInstance();
  // public static final Controls controls = new Controls();
  //private SendableChooser<Command> autoChooser;
  private final Timer seedImuTimer = new Timer();
  public static LED lights;
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  //private final Autos autos = new Autos(trailblazer);
  // public static OperatorOptions operatorOptions =
  // OperatorOptions.getInstance();

  private final Autos autos = new Autos(trailblazer);


  public Robot() {
    ArmManager.getInstance();
    // for (Command command : robotCommands.getPathplannerCommands()) {
    //   NamedCommands.registerCommand(command.getName(), command);
    // }
    // Command centerL1 = AutoBuilder.buildAuto("CenterL1");
    // Command centerL4 = AutoBuilder.buildAuto("CenterL4");

    // autoChooser = new SendableChooser<Command>();
    // // autoChooser.addOption("CenterL1", centerL1);
    // autoChooser.setDefaultOption("CenterL1", centerL1);
    // autoChooser.addOption("CenterL4", centerL4);

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
    // FmsSubsystem.getInstance().updateSimulation();
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
    //SmartDashboard.putData(autoChooser);
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

    autonomousCommand = autos.getAutoCommand();

    if(Utils.isSimulation()){
      localization.resetPose(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(90)));
    }
    DogLog.log("Selected Auto", autonomousCommand.getName());
    autonomousCommand.schedule();
    AutoAlign.getInstance().clearReefState();
  }

    // if (autoChooser.getSelected() != null)
    //   autoChooser.getSelected().schedule();
    // DogLog.log("Selected Auto", autoChooser.getSelected().getName());
//}

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
