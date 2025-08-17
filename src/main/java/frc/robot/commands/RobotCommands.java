package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.stateMachine.RobotManager;
import frc.robot.stateMachine.RobotState;

public class RobotCommands {

    private final RobotManager robot;
    private final Subsystem[] requirements;

    public RobotCommands(){
        this.robot = RobotManager.getInstance();
        var requirementsList = List.of(robot.armManager, robot.groundManager, robot.climber);
        requirements = requirementsList.toArray(Subsystem[]::new);
    }

    public Command scoreCommand() {
        return Commands.runOnce(robot::scoreRequest, requirements)
                .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.IDLE));
    }

    public Command algaeIntakeCommand() {
        return Commands.runOnce(robot::intakeAlgaeRequest, requirements)
                .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.IDLE));
    }

    public Command coralIntakeCommand() {
        return Commands.runOnce(robot::coralIntakeRequest, requirements)
                .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.IDLE));
    }

    public Command scoreLevelCommand() {
        return Commands.runOnce(robot::scoreLevelRequest, requirements)
                .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.WAIT_L1 || robot.getState() == RobotState.WAIT_L4));
    }

    public Command climbCommand() {
        return Commands.runOnce(robot::climbRequest, requirements)
                .andThen(Commands.waitUntil(() -> robot.getState() == RobotState.CLIMB));
    }
}
