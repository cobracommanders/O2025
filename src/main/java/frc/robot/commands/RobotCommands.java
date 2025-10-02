package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.autos.AutoBlocks;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private final Trailblazer trailblazer;
    public RobotCommands(Trailblazer trailblazer) {
        this.trailblazer = trailblazer;
    }

    private static final PoseErrorTolerance AUTOALIGN_CORAL_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(1.0), 1.0);
    public static final AutoConstraintOptions AUTOALIGN_CORAL_CONSTRAINTS = new AutoConstraintOptions(5.0, 360, 2.0, 720);

    public Command reefAlignCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().setState(DriveStates.REEF_ALIGN_TELEOP)).andThen(
                trailblazer.followSegment(
                        new AutoSegment(
                                AUTOALIGN_CORAL_CONSTRAINTS,
                                AUTOALIGN_CORAL_TOLERANCE,
                                new AutoPoint(() -> AutoAlign.getInstance().usedScoringPose)))
        ).andThen(print("Done aligning!"));
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
}
