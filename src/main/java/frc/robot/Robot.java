// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autos.Autos;
import frc.robot.autos.auto_path_commands.FourCoralNonProcessor;
import frc.robot.autos.auto_path_commands.FourCoralProcessor;
import frc.robot.autos.auto_path_commands.OneCoralClimberSideBackReef;
import frc.robot.commands.RobotCommands;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.Lights.LED;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.armManager.hand.Hand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.WinchSpeeds;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.util.MathHelpers;
import frc.robot.util.PhoenixSignalManager;

public class Robot extends TimedRobot {
    private static final Arm arm = new Arm();
    private static final Elevator elevator = new Elevator();
    private static final Hand hand = new Hand();

    public static ArmManager armManager = new ArmManager(
            hand,
            elevator,
            arm
    );

    private final CoralDetector coralDetector = CoralDetector.getInstance();

    private final RequestManager requestManager = new RequestManager(
            armManager,
            GroundManager.getInstance(),
            Climber.getInstance(),
            coralDetector::getState
    );
    private final DriveSubsystem swerve = DriveSubsystem.getInstance();
    private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
    private final RobotCommands robotCommands = new RobotCommands(trailblazer, requestManager);
    public static LocalizationSubsystem localization = LocalizationSubsystem.getInstance();

    private final Controls controls = new Controls(requestManager, robotCommands);

    public static LED lights;

    private final Autos autos = new Autos(trailblazer, requestManager, robotCommands);

    @Override
    public void robotPeriodic() {
        PhoenixSignalManager.refreshAll();

        CommandScheduler.getInstance().run();
        lights.periodic();

        if (FeatureFlags.useMechanismVisualizer.getAsBoolean()) {
            MechanismVisualizer.publishData();
        }

        swerve.setElevatorHeight(elevator.getHeight());
    }


    private final IntegerPublisher limelight_bl_throttle = NetworkTableInstance.getDefault().getTable("limelight-bl").getIntegerTopic("throttle_set").publish(PubSubOption.periodic(500));
    private final IntegerPublisher limelight_fl_throttle = NetworkTableInstance.getDefault().getTable("limelight-fl").getIntegerTopic("throttle_set").publish(PubSubOption.periodic(500));
    private final IntegerPublisher limelight_right_throttle = NetworkTableInstance.getDefault().getTable("limelight-right").getIntegerTopic("throttle_set").publish(PubSubOption.periodic(500));

    @Override
    public void disabledInit() {
        limelight_bl_throttle.set(200);
        limelight_fl_throttle.set(200);
        limelight_right_throttle.set(200);
    }

    @Override
    public void disabledExit() {
        limelight_bl_throttle.set(0);
        limelight_fl_throttle.set(0);
        limelight_right_throttle.set(0);
    }

    @Override
    public void robotInit() {
        DogLog.setOptions(new DogLogOptions().withLogExtras(false).withCaptureConsole(false));
//        DOGLOG.SETENABLED(ROBOT.ISSIMULATION());
        DogLog.setEnabled(true);

        FmsSubsystem.getInstance();
        lights = new LED();
        AutoAlign.getInstance();
        controls.configureDriveteamCommands();
        controls.configureDefaultCommands();
    }

//    private final Command autoCommand = new FourCoralNonProcessor(requestManager, trailblazer, robotCommands).getAutoCommand();
//    private final Command autoCommand = new FourCoralProcessor(requestManager, trailblazer, robotCommands).getAutoCommand();
//    private final Command autoCommand = new OneCoralClimberSideBackReef(requestManager, trailblazer, robotCommands).getAutoCommand();

    @Override
    public void autonomousInit() {
        if (Utils.isSimulation()) {
            Pose2d startingPose = new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(90));
            localization.resetPose(
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? MathHelpers.pathflip(startingPose) : startingPose
            );
        }

//        autoCommand.schedule();
        autos.getAutoCommand().schedule();
    }

    @Override
    public void teleopInit() {
        armManager.clearOverrideArmAcceleration();
        DriveSubsystem.getInstance().requestTeleop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        Climber.getInstance().setWinchSpeed(-WinchSpeeds.DEPLOYING);
    }

    @Override
    public void testExit() {
        Climber.getInstance().setWinchSpeed(0);
    }
}
