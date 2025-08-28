package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Ports.OIPorts;
import frc.robot.drivers.Xbox;
import frc.robot.fms.FmsSubsystem;
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
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYawFromFMS()));
        driver.POV180().onTrue(runOnce(() -> Elevator.getInstance().tickDown()));
        driver.POV0().onTrue(runOnce(() -> Elevator.getInstance().tickUp()));
        driver.POVMinus90().onTrue(runOnce(() -> IntakePivot.getInstance().tickUp()));
        driver.POV90().onTrue(runOnce(() -> IntakePivot.getInstance().tickDown()));
        driver.start().onTrue(Robot.robotCommands.resetToIdleCommand());
        driver.back().onTrue(Robot.robotCommands.groundIdleCommand());
       
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
        operator.POVMinus90().onTrue(Robot.robotCommands.setGroundAlgaeCommand());
        operator.POV180().onTrue(Robot.robotCommands.setLowReefAlgaeCommand());
        operator.back().onTrue(Robot.robotCommands.invertedHandoffToIdleCommand());
        operator.start().onTrue(Robot.robotCommands.resetToIdleCommand());
    }

    private static Controls instance;

    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
