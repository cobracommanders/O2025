package frc.robot;


import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.OIConstants;
import frc.robot.drivers.Xbox;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class Controls {
    /*
     *  BEGIN VARS FOR FUTURE USE
     *  
     *  private final double turtleSpeed = 0.1;
     *  private final double turtleAngularRate = Math.PI * 0.5;
     */

    
    private double angularRate = Constants.DrivertrainConstants.maxAngularRate;
    private double drivetrainSpeed = 0.75;

    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(Constants.DrivertrainConstants.maxSpeed * 0.1) // Deadband is handled on input
            .withRotationalDeadband(angularRate * 0.1);

     
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setTriggerThreshold(0.2);
        driver.setDeadzone(0.15);
        operator.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
    }

        private Supplier<SwerveRequest> controlStyle;

    private void newControlStyle() {
        controlStyle = () -> drive
                .withVelocityX((-(driver.leftY() * .5) * (driver.leftY() * .5) * (driver.leftY() * .5) * Constants.DrivertrainConstants.maxSpeed) * .7) // Drive
                                                                                                                         // forward
                                                                                                                         // -Y
                .withVelocityY((-(driver.leftX() * .5) * (driver.leftX() * .5) * (driver.leftX() * .5) * Constants.DrivertrainConstants.maxSpeed) * .7) // Drive
                                                                                                                         // left
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // X
                                                                                                                         // (left)
                .withRotationalRate((driver.rightX() * angularRate) * .1); // Drive counterclockwise with negative X
                                                                           // (left)
    }

    public void configureDefaultCommands() {
        newControlStyle();
        CommandSwerveDrivetrain.getInstance().setDefaultCommand(repeatingSequence( // Drivetrain will execute this
                                                                                   // command periodically
                runOnce(() -> CommandSwerveDrivetrain.getInstance()
                        .driveFieldRelative(new ChassisSpeeds(
                                -(driver.leftY() * drivetrainSpeed) * (driver.leftY()) * (driver.leftY()) * Constants.DrivertrainConstants.maxSpeed,
                                -(driver.leftX() * drivetrainSpeed) * (driver.leftX()) * (driver.leftX()) * Constants.DrivertrainConstants.maxSpeed,
                                driver.rightX() * angularRate)),
                        CommandSwerveDrivetrain.getInstance())));
    }

    public void configureDriverCommands() {
        driver.leftBumper().onTrue(Robot.robotCommands.algaeIntakeCommand());
        driver.leftTrigger().onTrue(Robot.robotCommands.coralIntakeCommand());
        driver.rightBumper().onTrue(Robot.robotCommands.scoreLevelCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
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
