package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Ports.OIPorts;
import frc.robot.drivers.Xbox;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.OperatorOptions.CoralMode;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.intake.IntakePivot;

public class Controls {
    DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    public final Xbox driver = new Xbox(OIPorts.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIPorts.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setTriggerThreshold(0.2);
        driver.setDeadzone(0.15);
        operator.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
    }

    public void configureDefaultCommands() {
        driveSubsystem.setDefaultCommand(
            driveSubsystem.run(
                () -> {
                    if (FmsSubsystem.getInstance().isTeleop()) {
                        driveSubsystem.setTeleopSpeeds(
                            driver.leftX(),
                            driver.leftY(),
                            driver.rightX());
                    }
                }));
    }

    public void configureDriverCommands() {
        driver.leftBumper().onTrue(Robot.robotCommands.algaeIntakeCommand());
        driver.leftTrigger().onTrue(Robot.robotCommands.coralIntakeCommand());
        driver.rightBumper().onTrue(Robot.robotCommands.prepareScoreWithHandoffCheckCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand().andThen(Robot.robotCommands.driveTeleopCommand()));
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYawFromFMS()));
        driver.POV180().onTrue(runOnce(() -> Elevator.getInstance().tickDown()));
        driver.POV0().onTrue(runOnce(() -> Elevator.getInstance().tickUp()));
        driver.POVMinus90().onTrue(runOnce(() -> IntakePivot.getInstance().tickUp()));
        driver.POV90().onTrue(runOnce(() -> IntakePivot.getInstance().tickDown()));
        driver.start().onTrue(Robot.robotCommands.resetToIdleCommand());
        driver.back().onTrue(Robot.robotCommands.groundIdleCommand());
        driver.B().onTrue(Robot.robotCommands.reefAlignCommand());
        driver.X().onTrue(Robot.robotCommands.driveTeleopCommand());
        driver.Y().onTrue(Robot.robotCommands.algaeAlignCommand());
       
    }

    public void configureOperatorCommands() {
        operator.leftBumper().onTrue(Robot.robotCommands.setProcessorCommand());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
        operator.rightBumper().onTrue(Robot.robotCommands.setBargeCommand());
        operator.Y().onTrue(Robot.robotCommands.setL3Command());
        operator.B().onTrue(Robot.robotCommands.setL4Command());
        operator.X().onTrue(Robot.robotCommands.setL2Command());
        operator.A().onTrue(Robot.robotCommands.setL1Command());
        operator.POV0().onTrue(Robot.robotCommands.setHighReefAlgaeCommand());
        operator.POV90().onTrue(Robot.robotCommands.setGroundAlgaeCommand());
        operator.rightStick().onTrue(runOnce(() -> setCoralMode()));
        operator.leftStick().onTrue(runOnce(() -> setNormalMode()));
        operator.POV180().onTrue(Robot.robotCommands.setLowReefAlgaeCommand());
        operator.back().onTrue(Robot.robotCommands.invertedHandoffToIdleCommand());
        operator.start().onTrue(Robot.robotCommands.resetToIdleCommand());
    }

    public void setCoralMode() {
        RequestManager.getInstance().operatorOptions.coralMode = OperatorOptions.CoralMode.CORAL_MODE;
        DogLog.log("Control/Coral Mode Enabled", "CORAL");
    }

    public void setNormalMode() {
        RequestManager.getInstance().operatorOptions.coralMode = OperatorOptions.CoralMode.NORMAL_MODE;
        DogLog.log("Control/Coral Mode Enabled", "NORMAL");
    }

    private static Controls instance;

    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
