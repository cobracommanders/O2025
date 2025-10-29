package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Ports.OIPorts;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.commands.RobotCommands;
import frc.robot.drivers.Xbox;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.armScheduler.ArmScheduler;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.intake.IntakePivot;
import frc.robot.vision.VisionSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.DriverStation;

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
        driver.A().onTrue(runOnce(() -> DriveSubsystem.getInstance().setYawFromFMS()));

        // Reset superstructure and clear game piece
        driver.start().onTrue(requestManager.resetArmGamePieceAndIdle());

        // Score Coral
        operator.rightTrigger()
                // whileTrue will cancel the command when the button is released
                .whileTrue(Commands.either(
                        requestManager.scoreL1(() -> driver.rightTrigger().getAsBoolean()).asProxy(),
                        robotCommands.teleopReefAlignAndScore(driver::isStickActive,
                                () -> driver.B().getAsBoolean(),
                                false),
                        () -> OperatorOptions.getInstance().isCoralScoringL1()
                ))
                // onFalse will reset the superstructure if the button is released (likely means the command is cancelled)
                // Only resets if the robot is far away from the reef and not likely to score again soon
                .onFalse(requestManager.idleAll().onlyIf(() -> AutoAlign.getInstance().approximateDistanceToReef() > 0.125));


        driver.Y().onTrue(
                Commands.either(
                        requestManager.algaeNetScore(() -> requestManager.netRobotSide()),
                        requestManager.algaeProcessorScore(() -> driver.X().getAsBoolean()),
                        () -> operatorOptions.algaeScoreLocation == OperatorOptions.AlgaeScoreLocation.BARGE
                )
        );
        driver.X().onTrue(requestManager.armCommands.executeAlgaeNetScoreAndAwaitIdle());

        operator.leftTrigger()
        // whileTrue will cancel the command when the button is released
        .whileTrue(Commands.either(
                requestManager.scoreL1(() -> driver.rightTrigger().getAsBoolean()).asProxy(),
                robotCommands.teleopReefAlignAndScore(driver::isStickActive,
                        () -> driver.B().getAsBoolean(),
                        true),
                () -> OperatorOptions.getInstance().isCoralScoringL1()
        ))
        // onFalse will reset the superstructure if the button is released (likely means the command is cancelled)
        // Only resets if the robot is far away from the reef and not likely to score again soon
        .onFalse(requestManager.idleAll().onlyIf(() -> AutoAlign.getInstance().approximateDistanceToReef() > 0.125));

        // Fix drivetrain state
//        driver.X().onTrue(runOnce(() -> {
//            Command currentCommand = driveSubsystem.getCurrentCommand();
//            if (currentCommand != null) {
//                currentCommand.cancel();
//            }
//        }).andThen(robotCommands.driveTeleopCommand()));

        // Align to Reef Algae
        // driver.Y().onTrue(robotCommands.algaeAlignCommand());

        // Algae Intake
        //driver.leftBumper().onTrue(requestManager.reefAlgaeIntake());
        driver.rightTrigger().and((operator.leftTrigger().negate().and(operator.rightTrigger().negate()))).onTrue(Commands.either(
                requestManager.groundAlgaeIntake(),
                requestManager.reefAlgaeIntake(),
                () -> operatorOptions.algaeIntakeLevel == OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE
        )).onFalse(
                requestManager.resetGroundAlgaeIntake()
        );

        // Tick Intake Pivot
        driver.POVMinus90().onTrue(runOnce(() -> IntakePivot.getInstance().tickUp()));
        driver.POV90().onTrue(runOnce(() -> IntakePivot.getInstance().tickDown()));


        /* ******** OPERATOR ******** */
        operator.leftBumper().onTrue(operatorOptions.setProcessorCommand());
//        driver.leftBumper().and(driver.rightBumper()).onTrue(requestManager.climbRequest());
        operator.rightBumper().onTrue(operatorOptions.setBargeCommand());
        operator.Y().onTrue(operatorOptions.setL3Command());
        operator.B().onTrue(operatorOptions.setL4Command());
        operator.X().onTrue(operatorOptions.setL2Command());
        operator.A().onTrue(operatorOptions.setL1Command());
        operator.POV0().onTrue(operatorOptions.setHighReefAlgaeCommand());
        //operator.POVMinus90().onTrue(requestManager.prepareCoralScoreAndAwaitReady().andThen(robotCommands.driveTeleopCommand()));
        operator.POV90().onTrue(operatorOptions.setGroundAlgaeCommand());
        operator.POV180().onTrue(operatorOptions.setLowReefAlgaeCommand());
        operator.back().onTrue(requestManager.climbRequest());
        operator.start().onTrue(requestManager.resetArmGamePieceAndIdle());
    }
}
