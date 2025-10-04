package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.drivetrain.DriveStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private final Trailblazer trailblazer;
    private final ArmManager armManager;

    public RobotCommands(Trailblazer trailblazer, ArmManager armManager) {
        this.armManager = armManager;
        this.trailblazer = trailblazer;
    }

    private static final PoseErrorTolerance AUTOALIGN_CORAL_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(1.0), 1.0);
    public static final AutoConstraintOptions AUTOALIGN_CORAL_CONSTRAINTS = new AutoConstraintOptions(5.0, 360, 2.5, 720);

    private static final PoseErrorTolerance AUTOALIGN_CORAL_TOLERANCE_DRIVEUP = new PoseErrorTolerance(Units.inchesToMeters(4.0), 4.0);
    public static final AutoConstraintOptions AUTOALIGN_CORAL_CONSTRAINTS_DRIVEUP = new AutoConstraintOptions(5.0, 360, 3.0, 720);

    private final Transform2d ARM_DOWN_OFFSET_LEFT = new Transform2d(0.0, -0.4, Rotation2d.kZero);
    private final Transform2d ARM_DOWN_OFFSET_RIGHT = new Transform2d(0.0, 0.4, Rotation2d.kZero);
    private final Transform2d INITIAL_DRIVE_OFFSET_LEFT = new Transform2d(0.0, -0.5, Rotation2d.kZero);
    private final Transform2d INITIAL_DRIVE_OFFSET_RIGHT = new Transform2d(0.0, 0.5, Rotation2d.kZero);

    public Command reefAlignCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().requestReefAlign())
                .andThen(
                        trailblazer.followSegment(
                                new AutoSegment(
                                        AUTOALIGN_CORAL_CONSTRAINTS_DRIVEUP,
                                        AUTOALIGN_CORAL_TOLERANCE_DRIVEUP,
                                        new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                            case LEFT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_LEFT);
                                            case RIGHT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_RIGHT);
                                        })
                                )
                        )
                ).andThen(
                        trailblazer.followSegment(
                                        new AutoSegment(
                                                AUTOALIGN_CORAL_CONSTRAINTS,
                                                AUTOALIGN_CORAL_TOLERANCE,
                                                new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                                    case LEFT ->
                                                            AutoAlign.getInstance().usedScoringPose.transformBy(ARM_DOWN_OFFSET_LEFT);
                                                    case RIGHT ->
                                                            AutoAlign.getInstance().usedScoringPose.transformBy(ARM_DOWN_OFFSET_RIGHT);
                                                })
                                        )
                                )
                                .unless(armManager::isArmUp)
                ).andThen(
                        trailblazer.followSegment(
                                new AutoSegment(
                                        AUTOALIGN_CORAL_CONSTRAINTS,
                                        AUTOALIGN_CORAL_TOLERANCE,
                                        new AutoPoint(() -> AutoAlign.getInstance().usedScoringPose)))
                )
                .finallyDo(() -> DriveSubsystem.getInstance().requestTeleop());
    }

    public Command algaeAlignCommand() {
        return runOnce(() -> AutoAlign.getInstance().setAlgaeIntakingOffset(ReefSideOffset.ALGAE_INTAKING)).andThen(runOnce(() -> DriveSubsystem.getInstance().requestAlgaeAlign()));
    }

    public Command driveTeleopCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().requestTeleop());
    }
}
