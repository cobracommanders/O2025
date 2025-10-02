package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Ports.OIPorts;
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

        /* ******** DRIVER ******** */
        driver.leftBumper().onTrue(Commands.either(
                requestManager.groundAlgaeIntake(),
                requestManager.reefAlgaeIntake(),
                () -> OperatorOptions.getInstance().algaeIntakeLevel == OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE
        ));

        driver.leftTrigger().onTrue(requestManager.coralIntakeUntilPiece());
        driver.rightTrigger().onTrue(requestManager.executeCoralScoreAndAwaitIdleOrAuto());
//        driver.rightBumper().onTrue(Robot.robotCommands.prepareScoreWithHandoffCheckCommand());
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYawFromFMS()));
        // driver.POV180().onTrue(runOnce(() -> Robot.armManager.elevatorTickDown()));
        // driver.POV0().onTrue(runOnce(() -> Robot.armManager.elevatorTickUp()));
        driver.POVMinus90().onTrue(runOnce(() -> IntakePivot.getInstance().tickUp()));
        driver.POV90().onTrue(runOnce(() -> IntakePivot.getInstance().tickDown()));
        driver.start().onTrue(requestManager.resetArmGamePieceAndIdle());
//        driver.B().onTrue(Robot.robotCommands.reefAlignCommand());
        driver.rightBumper().onTrue(robotCommands.reefAlignCommand().andThen(requestManager.executeCoralScoreAndAwaitIdleOrAuto()));
        driver.X().onTrue(robotCommands.driveTeleopCommand());
        driver.Y().onTrue(robotCommands.algaeAlignCommand());


        /* ******** OPERATOR ******** */
        operator.leftBumper().onTrue(robotCommands.setProcessorCommand());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(requestManager.climbRequest());
        operator.rightBumper().onTrue(robotCommands.setBargeCommand());
        operator.Y().onTrue(robotCommands.setL3Command());
        operator.B().onTrue(robotCommands.setL4Command());
        operator.X().onTrue(robotCommands.setL2Command());
        operator.A().onTrue(robotCommands.setL1Command());
        operator.POV0().onTrue(robotCommands.setHighReefAlgaeCommand());
        operator.POVMinus90().onTrue(requestManager.prepareCoralScoreAndAwaitReady().andThen(robotCommands.driveTeleopCommand()));
        operator.POV90().onTrue(robotCommands.setGroundAlgaeCommand());
        operator.POV180().onTrue(robotCommands.setLowReefAlgaeCommand());
        operator.back().onTrue(requestManager.invertedHandoffRequest());
        operator.start().onTrue(requestManager.resetArmGamePieceAndIdle());
    }
}
