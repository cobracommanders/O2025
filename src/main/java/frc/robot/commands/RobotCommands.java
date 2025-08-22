package frc.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.stateMachine.RobotManager;
import frc.robot.stateMachine.RobotState;

public class RobotCommands {

    private final RobotManager robotManager;
    private final Subsystem[] requirements;

    public RobotCommands(){
        this.robotManager = RobotManager.getInstance();
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
            waitForState(null)
        };
    }
    public Command waitForState(RobotState state) {
        return robotManager.waitForState(state).withName("waitForState/" + state.toString());
    }
    public Command scoreCommand() {
        return Commands.runOnce(robotManager::scoreRequest, requirements)
                .andThen(Commands.waitUntil(() -> robotManager.getState() == RobotState.IDLE)).withName("score");
    }

    public Command algaeIntakeCommand() {
        return Commands.runOnce(robotManager::intakeAlgaeRequest, requirements)
                .andThen(Commands.waitUntil(() -> robotManager.getState() == RobotState.IDLE)).withName("algaeIntake");
    }

    public Command coralIntakeCommand() {
        return Commands.runOnce(robotManager::coralIntakeRequest, requirements)
                .andThen(Commands.waitUntil(() -> robotManager.getState() == RobotState.IDLE)).withName("coralIntake");
    }

    public Command prepareScoreCommand() {
        return Commands.runOnce(robotManager::scoreLevelRequest, requirements).withName("prepareScore");
                //.andThen(Commands.waitUntil(() -> robot.getState() == RobotState.WAIT_L1 || robot.getState() == RobotState.WAIT_L4));
    }

    public Command climbCommand() {
        return Commands.runOnce(robotManager::climbRequest, requirements)
                .andThen(Commands.waitUntil(() -> robotManager.getState() == RobotState.CLIMB)).withName("climb");
    }

    public Command setProcessorCommand(){
        return Commands.runOnce(() -> robotManager.setProcessor()).withName("setProcessor");
    }

    public Command setBargeCommand(){
        return Commands.runOnce(() -> robotManager.setBarge()).withName("setBarge");
    }

    public Command setL1Command(){
        return Commands.runOnce(() -> robotManager.setL1()).withName("setL1");
    }

    public Command setL2Command(){
        return Commands.runOnce(() -> robotManager.setL2()).withName("setL2");
    }

    public Command setL3Command(){
        return Commands.runOnce(() -> robotManager.setL3()).withName("setL3");
    }

    public Command setL4Command(){
        return Commands.runOnce(() -> robotManager.setL4()).withName("setL4");
    }

    public Command setHighReefAlgaeCommand(){
        return Commands.runOnce(() -> robotManager.setHighReefAlgae()).withName("setHighReefAlgae");
    }

    public Command setLowReefAlgaeCommand(){
        return Commands.runOnce(() -> robotManager.setLowReefAlgae()).withName("setLowReefAlgae");
    }

    public Command setGroundAlgaeCommand(){
        return Commands.runOnce(() -> robotManager.setGroundAlgae()).withName("setGroundAlgae");
    }

    public Command resetToIdleCommand(){
        return Commands.runOnce(() -> robotManager.resetToIdleRequest()).withName("resetToIdle");
    }
}
