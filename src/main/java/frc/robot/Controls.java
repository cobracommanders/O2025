package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Ports.OIPorts;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.commands.RobotCommands;
import frc.robot.drivers.Xbox;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.intake.IntakePivot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Controls {
    DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    private final Xbox driver = new Xbox(OIPorts.DRIVER_CONTROLLER_ID);
    private final Xbox operator = new Xbox(OIPorts.OPERATOR_CONTROLLER_ID);

    private final RequestManager requestManager;
    private final RobotCommands robotCommands;

    public Controls(RequestManager requestManager, RobotCommands robotCommands) {
        this.requestManager = requestManager;
        this.robotCommands = robotCommands;

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

    public void configureDriveteamCommands() {
        var operatorOptions = OperatorOptions.getInstance();

        /* ******** DRIVER ******** */
        // Intake Coral
        driver.leftTrigger().onTrue(requestManager.coralIntakeUntilPiece());

        // Reset Gyro
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYawFromFMS()));

        // Reset superstructure and clear game piece
        driver.start().onTrue(requestManager.resetArmGamePieceAndIdle());

        // Score Coral
        driver.rightBumper()
                // whileTrue will cancel the command when the button is released
                .whileTrue(robotCommands.teleopReefAlignAndScore(driver::isStickActive))
                // onFalse will reset the superstructure if the button is released (likely means the command is cancelled)
                // Only resets if the robot is far away from the reef and not likely to score again soon
                .onFalse(requestManager.idleArm().onlyIf(() -> AutoAlign.getInstance().approximateDistanceToReef() > 0.125));

        // Fix drivetrain state
        driver.X().onTrue(runOnce(() -> {
            Command currentCommand = driveSubsystem.getCurrentCommand();
            if (currentCommand != null) {
                currentCommand.cancel();
            }
        }).andThen(robotCommands.driveTeleopCommand()));

        // Align to Reef Algae
        // driver.Y().onTrue(robotCommands.algaeAlignCommand());

        // Algae Intake
        driver.leftBumper().onTrue(Commands.either(
                requestManager.groundAlgaeIntake(),
                requestManager.reefAlgaeIntake(),
                () -> operatorOptions.algaeIntakeLevel == OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE
        ));

        // Tick Intake Pivot
        driver.POVMinus90().onTrue(runOnce(() -> IntakePivot.getInstance().tickUp()));
        driver.POV90().onTrue(runOnce(() -> IntakePivot.getInstance().tickDown()));


        /* ******** OPERATOR ******** */
        operator.leftBumper().onTrue(operatorOptions.setProcessorCommand());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(requestManager.climbRequest());
        operator.rightBumper().onTrue(operatorOptions.setBargeCommand());
        operator.Y().onTrue(operatorOptions.setL3Command());
        operator.B().onTrue(operatorOptions.setL4Command());
        operator.X().onTrue(operatorOptions.setL2Command());
        operator.A().onTrue(operatorOptions.setL1Command());
        operator.POV0().onTrue(operatorOptions.setHighReefAlgaeCommand());
        operator.POVMinus90().onTrue(requestManager.prepareCoralScoreAndAwaitReady().andThen(robotCommands.driveTeleopCommand()));
        operator.POV90().onTrue(operatorOptions.setGroundAlgaeCommand());
        operator.POV180().onTrue(operatorOptions.setLowReefAlgaeCommand());
        operator.back().onTrue(requestManager.invertedHandoffRequest());
        operator.start().onTrue(requestManager.resetArmGamePieceAndIdle());
    }
}
