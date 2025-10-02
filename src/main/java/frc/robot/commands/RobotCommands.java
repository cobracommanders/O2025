package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private RobotCommands() {}

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
