package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefSideOffset;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private final Trailblazer trailblazer;
    private final RequestManager requestManager;

    public RobotCommands(Trailblazer trailblazer, RequestManager requestManager) {
        this.requestManager = requestManager;
        this.trailblazer = trailblazer;
    }

    private static final PoseErrorTolerance CORAL_SCORE_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(0.75), 1.0);
    public static final AutoConstraintOptions EXTENDED_DRIVE_CONSTRAINTS = new AutoConstraintOptions(5.0, 360, 2.0, 720);

    private final Transform2d AWAIT_ARM_UP_LEFT_OFFSET = new Transform2d(0.0, -0.2, Rotation2d.kZero);
    private final Transform2d AWAIT_ARM_UP_RIGHT_OFFSET = new Transform2d(0.0, 0.2, Rotation2d.kZero);
    private final Transform2d DRIVE_BACK_AFTER_SCORE_LEFT_OFFSET = new Transform2d(0.0, -0.5, Rotation2d.kZero);
    private final Transform2d DRIVE_BACK_AFTER_SCORE_RIGHT_OFFSET = new Transform2d(0.0, 0.5, Rotation2d.kZero);

    public Command teleopReefAlignAndScore(BooleanSupplier backupDriveInterrupt) {
        return parallel(
                        requestManager.prepareCoralScoreAndAwaitReady(),
                        trailblazer.followSegment(
                                new AutoSegment(
                                        EXTENDED_DRIVE_CONSTRAINTS,
                                        CORAL_SCORE_TOLERANCE,
                                        new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                            case LEFT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(AWAIT_ARM_UP_LEFT_OFFSET);
                                            case RIGHT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(AWAIT_ARM_UP_RIGHT_OFFSET);
                                        })
                                )
                        ).until(requestManager::isArmReadyToScoreCoral)
                ).andThen(
                        trailblazer.followSegment(
                                new AutoSegment(
                                        EXTENDED_DRIVE_CONSTRAINTS,
                                        CORAL_SCORE_TOLERANCE,
                                        new AutoPoint(() -> AutoAlign.getInstance().usedScoringPose)
                                )
                        ),
                        requestManager.executeCoralScoreAndAwaitComplete(),
                        trailblazer.followSegment(
                                new AutoSegment(
                                        EXTENDED_DRIVE_CONSTRAINTS,
                                        CORAL_SCORE_TOLERANCE,
                                        new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                            case LEFT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(DRIVE_BACK_AFTER_SCORE_LEFT_OFFSET);
                                            case RIGHT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(DRIVE_BACK_AFTER_SCORE_RIGHT_OFFSET);
                                        })
                                )
                        ).until(backupDriveInterrupt)
                )
                .beforeStarting(() -> DriveSubsystem.getInstance().requestReefAlign())
                .finallyDo((i) -> DriveSubsystem.getInstance().requestTeleop());
    }

    public Command algaeAlignCommand() {
        return runOnce(() -> AutoAlign.getInstance().setAlgaeIntakingOffset(ReefSideOffset.ALGAE_INTAKING)).andThen(runOnce(() -> DriveSubsystem.getInstance().requestAlgaeAlign()));
    }

    public Command driveTeleopCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().requestTeleop());
    }
}
