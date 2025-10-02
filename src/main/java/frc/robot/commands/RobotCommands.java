package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.stateMachine.OperatorOptions;
import frc.robot.stateMachine.OperatorOptions.ScoreLocation;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private final OperatorOptions operatorOptions = OperatorOptions.getInstance();

    private RobotCommands() {}

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

    public Command reefAlignCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.REEF_ALIGN_TELEOP)).andThen(DriveSubsystem.getInstance().waitForState(DriveStates.TELEOP)).withName("teleop align");
    }

    public Command autoReefAlignCommand() {
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
