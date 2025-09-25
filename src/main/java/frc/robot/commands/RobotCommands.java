package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.RequestManagerState;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.ground_manager.GroundManager;
import frc.robot.subsystems.ground_manager.GroundManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class RobotCommands {
    private final RequestManager robotManager = RequestManager.getInstance();
    private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

    private RobotCommands() {
    }

    public Command waitForAllIdle() {
        return GroundManager.getInstance().waitForState(GroundManagerStates.IDLE)
                .alongWith(waitUntil(() -> Robot.armManager.isIdleState()))
                .withName("waitForState/ALL_IDLE");
    }

    public Command awaitCoralReadyToScore() {
        return waitUntil(() -> Robot.armManager.isReadyToScoreCoral()).withName("awaitReadyToScoreCoral");
    }

    public Command awaitCoralFinishedScoring() {
        return waitUntil(() -> Robot.armManager.getState().isCoralScoreState() && Robot.armManager.atPosition());
    }

    public Command scoreCommand() {
        return runOnce(robotManager::scoreRequest).withName("score");
    }

    public Command algaeIntakeCommand() {
        return runOnce(robotManager::intakeAlgaeRequest).withName("algaeIntake");
    }

    public Command coralIntakeCommand() {
        return runOnce(robotManager::coralIntakeRequest).withName("coralIntake");
    }

    public Command autoLollipopIntakeCommand() {
        return runOnce(robotManager::lollipopIntakeRequest).withName("lollipopIntake");
    }

    public Command prepareScoreCommand() {
        return runOnce(robotManager::scoreLevelRequest).withName("prepareScore");
    }

    public Command climbCommand() {
        return runOnce(robotManager::climbRequest)
                .andThen(waitUntil(() -> robotManager.getState() == RequestManagerState.CLIMB))
                .withName("climb");
    }

    public Command invertedHandoffCommand() {
        return runOnce(robotManager::invertedHandoffRequest)
                .andThen(robotManager.waitForState(RequestManagerState.INDEPENDENT))
                .withName("invertedHandoff");
    }

    public Command handoffCommand() {
        return runOnce(robotManager::handoffRequest)
                .andThen(robotManager.waitForState(RequestManagerState.INDEPENDENT))
                .withName("handoff");
    }

    public Command resetToIdleCommand() {
        return runOnce(robotManager::resetToIdleRequest).withName("resetToIdle");
    }

    public Command groundIdleCommand() {
        return runOnce(robotManager::groundIdleRequest).withName("groundIdle");
    }

    public Command setProcessorCommand() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.PROCESSOR).withName("setProcessor");
    }

    public Command setBargeCommand() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.BARGE).withName("setBarge");
    }

    public Command setL1Command() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.L1).withName("setL1");
    }

    public Command setL2Command() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.L2).withName("setL2");
    }

    public Command setL3Command() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.L3).withName("setL3");
    }

    public Command setL4Command() {
        return runOnce(() -> operatorOptions.scoreLocation = ScoreLocation.L4).withName("setL4");
    }

    public Command setHighReefAlgaeCommand() {
        return runOnce(() -> operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.HIGH_REEF).withName("setHighReefAlgae");
    }

    public Command setLowReefAlgaeCommand() {
        return runOnce(() -> operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.LOW_REEF).withName("setLowReefAlgae");
    }

    public Command setGroundAlgaeCommand() {
        return runOnce(() -> operatorOptions.algaeIntakeLevel = OperatorOptions.AlgaeIntakeLevel.GROUND_ALGAE).withName("setGroundAlgae");
    }

    public Command prepareScoreWithHandoffCheckCommand() {
        return new ConditionalCommand(
                handoffCommand().andThen(prepareScoreCommand(), groundIdleCommand()),
                prepareScoreCommand(),
                () -> CoralDetector.getInstance().hasCoral() &&
                        (OperatorOptions.getInstance().scoreLocation != ScoreLocation.BARGE &&
                                OperatorOptions.getInstance().scoreLocation != ScoreLocation.PROCESSOR &&
                                OperatorOptions.getInstance().scoreLocation != ScoreLocation.L1))
                .withName("prepareScoreWithHandoffCheck");

    }

    public Command reefAlignCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.REEF_ALIGN_TELEOP)).andThen(DriveSubsystem.getInstance().waitForState(DriveStates.TELEOP).andThen(scoreCommand())).withName("teleop align");
    }

    public Command autoReefAlignCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.REEF_ALIGN_TELEOP)).andThen(DriveSubsystem.getInstance().waitForState(DriveStates.AUTO)).andThen(scoreCommand()).withName("auto align");
    }

    public Command autoReefAlignCommandNoScore() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.REEF_ALIGN_TELEOP)).andThen(DriveSubsystem.getInstance().waitForState(DriveStates.AUTO)).withName("auto align");
    }

    public Command algaeAlignCommand() {
        return runOnce(() -> AutoAlign.getInstance().setAlgaeIntakingOffset(ReefSideOffset.ALGAE_INTAKING)).andThen(runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.ALGAE_ALIGN_TELEOP)));
    }

    public Command driveTeleopCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.TELEOP));
    }

    private static RobotCommands instance;

    public static RobotCommands getInstance() {
        if (instance == null)
            instance = new RobotCommands(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
