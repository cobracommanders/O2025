// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.commands.RobotCommands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.Lights.LED;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class Robot extends TimedRobot {
    private static final Arm arm = new Arm();
    private static final Elevator elevator = new Elevator();
    private static final Hand hand = new Hand();

    public static ArmManager armManager = new ArmManager(
            hand,
            elevator,
            arm
    );
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
        // for (Command command : robotCommands.getPathplannerCommands()) {
        //   NamedCommands.registerCommand(command.getName(), command);
        // }
        // Command centerL1 = AutoBuilder.buildAuto("CenterL1");
        // Command centerL4 = AutoBuilder.buildAuto("CenterL4");

        // autoChooser = new SendableChooser<Command>();
        // // autoChooser.addOption("CenterL1", centerL1);
        // autoChooser.setDefaultOption("CenterL1", centerL1);
        // autoChooser.addOption("CenterL4", centerL4);
        // autoChooser.addOption("Center1", center1);
        // autoChooser.addOption("CenterL3", centerL3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        lights.periodic();
        MechanismVisualizer.publishData();
        // FmsSubsystem.getInstance().updateSimulation();

        swerve.setElevatorHeight(elevator.getHeight());
    }

    @Override
    public void robotInit() {
        // CommandScheduler.getInstance().onCommandInitialize((command)-> DogLog.log("CommandScheduler/Scheduled Commands", command.getName()));
        FmsSubsystem.getInstance();
        //SmartDashboard.putData(autoChooser);
        lights = new LED();
        AutoAlign.getInstance();
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
    public void teleopInit() {
        Controls.getInstance().configureDriverCommands();
        Controls.getInstance().configureOperatorCommands();
        Controls.getInstance().configureDefaultCommands();
  DriveSubsystem.getInstance().setState(DriveStates.TELEOP);
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
