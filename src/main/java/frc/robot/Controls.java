package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Ports.OIPorts;
import frc.robot.drivers.Xbox;
import frc.robot.fms.FmsSubsystem;
import frc.robot.subsystems.armManager.elevator.Elevator;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

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
        driver.rightBumper().onTrue(Robot.robotCommands.scoreLevelCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
        driver.A().onTrue(runOnce(()->CommandSwerveDrivetrain.getInstance().setYaw(Rotation2d.kZero)));
        driver.POV180().onTrue(runOnce(() -> Elevator.getInstance().tickDown()));
        driver.POV0().onTrue(runOnce(() -> Elevator.getInstance().tickUp()));
        driver.start().onTrue(runOnce(()->Robot.robotManager.resetToIdleRequest()));
    }

    public void configureOperatorCommands(){
      operator.leftBumper().onTrue(runOnce(()->Robot.robotManager.setProcessor()));
      operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
      operator.rightBumper().onTrue(runOnce(()->Robot.robotManager.setBarge()));
      operator.Y().onTrue(runOnce(()->Robot.robotManager.setL3()));
      operator.B().onTrue(runOnce(()->Robot.robotManager.setL4()));
      operator.X().onTrue(runOnce(()->Robot.robotManager.setL2()));
      operator.A().onTrue(runOnce(()->Robot.robotManager.setL1()));
      operator.POV0().onTrue(runOnce(()->Robot.robotManager.setHighReefAlgae()));
      operator.POV90().onTrue(runOnce(()->Robot.robotManager.setGroundAlgae()));
      operator.POVMinus90().onTrue(runOnce(()->Robot.robotManager.setGroundAlgae()));
      operator.POV180().onTrue(runOnce(()->Robot.robotManager.setLowReefAlgae()));
    }



    private static Controls instance;

    public static Controls getInstance() {
      if (instance == null) instance = new Controls(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
