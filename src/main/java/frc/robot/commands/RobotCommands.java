package frc.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.stateMachine.RobotManager;
import frc.robot.stateMachine.RobotState;
import frc.robot.stateMachine.RequestManager;
import frc.robot.stateMachine.RequestManagerStates;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.ground_manager.GroundManagerStates;

public class RobotCommands {

    private final RequestManager robotManager;
    private final Subsystem[] requirements;

    public RobotCommands() {
        this.robotManager = RequestManager.getInstance();
        var requirementsList = List.of(robotManager.armManager, robotManager.groundManager, robotManager.climber);
        requirements = requirementsList.toArray(Subsystem[]::new);
    }

    public Command[] getPathplannerCommands() {
        return new Command[] {
                scoreCommand(),
                algaeIntakeCommand(),
                coralIntakeCommand(),
                prepareScoreCommand(),
                setProcessorCommand(),
                setBargeCommand(),
                setL1Command(),
                setL2Command(),
                setL3Command(),
                setL4Command(),
                setHighReefAlgaeCommand(),
                setLowReefAlgaeCommand(),
                setGroundAlgaeCommand(),
                waitForState(RequestManagerStates.INDEPENDENT),
                waitForGroundReady(),
                waitForWaitL4()
        };
    }

    public Command waitForState(RequestManagerStates state) {
        return robotManager.waitForState(state).withName("waitForState/" + state.toString());
    }

    public Command waitForGroundReady() {
        return robotManager.groundManager.waitForState(GroundManagerStates.WAIT_SCORE_L1)
                .withName("waitForGroundState/WAIT_SCORE_L1");
    }

    public Command waitForWaitL4() {
        return robotManager.armManager.waitForState(ArmManagerStates.WAIT_L4)
                .withName("waitForArmState/WAIT_L4");
    }

    public Command scoreCommand() {
        return Commands.runOnce(robotManager::scoreRequest, requirements).withName("score");
    }

    public Command algaeIntakeCommand() {
        return Commands.runOnce(robotManager::intakeAlgaeRequest, requirements).withName("algaeIntake");
    }

    public Command coralIntakeCommand() {
        return Commands.runOnce(robotManager::coralIntakeRequest, requirements).withName("coralIntake");
    }

    public Command prepareScoreCommand() {
        return Commands.runOnce(robotManager::scoreLevelRequest, requirements).withName("prepareScore");
        // .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.WAIT_L1 ||
        // robot.getState() == RobotState.WAIT_L4));
    }

    public Command climbCommand() {
        return Commands.runOnce(robotManager::climbRequest, requirements)
                .andThen(Commands.waitUntil(() -> robotManager.getState() == RequestManagerStates.CLIMB))
                .withName("climb");
    }

    public Command setProcessorCommand() {
        return Commands.runOnce(() -> robotManager.setProcessor()).withName("setProcessor");
    }

    public Command setBargeCommand() {
        return Commands.runOnce(() -> robotManager.setBarge()).withName("setBarge");
    }

    public Command setL1Command() {
        return Commands.runOnce(() -> robotManager.setL1()).withName("setL1");
    }

    public Command setL2Command() {
        return Commands.runOnce(() -> robotManager.setL2()).withName("setL2");
    }

    public Command setL3Command() {
        return Commands.runOnce(() -> robotManager.setL3()).withName("setL3");
    }

    public Command setL4Command() {
        return Commands.runOnce(() -> robotManager.setL4()).withName("setL4");
    }

    public Command invertedHandoffCommand() {
        return Commands.runOnce(() -> robotManager.invertedHandoffRequest()).withName("invertedHandoff");
    }

    public Command handoffCommand() {
        return Commands.runOnce(() -> robotManager.handoffRequest()).withName("handoff");
    }

    public Command setHighReefAlgaeCommand() {
        return Commands.runOnce(() -> robotManager.setHighReefAlgae()).withName("setHighReefAlgae");
    }

    public Command setLowReefAlgaeCommand() {
        return Commands.runOnce(() -> robotManager.setLowReefAlgae()).withName("setLowReefAlgae");
    }

    public Command setGroundAlgaeCommand() {
        return Commands.runOnce(() -> robotManager.setGroundAlgae()).withName("setGroundAlgae");
    }

    public Command resetToIdleCommand() {
        return Commands.runOnce(() -> robotManager.resetToIdleRequest()).withName("resetToIdle");
    }

    public Command groundIdleCommand() {
        return Commands.runOnce(() -> robotManager.groundIdleRequest()).withName("groundIdle");
    }

    public Command invertedHandoffToIdleCommand() {
        return invertedHandoffCommand()
                .andThen(robotManager.waitForState(RequestManagerStates.INDEPENDENT).andThen(resetToIdleCommand()));
    }

    public Command prepareScoreWithHandoffCheckCommand() {
        return new ConditionalCommand(
                handoffCommand().andThen(robotManager.waitForState(RequestManagerStates.INDEPENDENT))
                        .andThen(prepareScoreCommand()).andThen(groundIdleCommand()),
                prepareScoreCommand(),
                () -> robotManager.coralDetector.hasCoral() &&
                        (robotManager.operatorOptions.scoreLocation != ScoreLocation.BARGE &&
                                robotManager.operatorOptions.scoreLocation != ScoreLocation.PROCESSOR &&
                                robotManager.operatorOptions.scoreLocation != ScoreLocation.L1))
                .withName("prepareScoreWithHandoffCheck");

    }
}
