package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotCommands {
    private final Trailblazer trailblazer;
    private final RequestManager requestManager;

    public RobotCommands(Trailblazer trailblazer, RequestManager requestManager) {
        this.requestManager = requestManager;
        this.trailblazer = trailblazer;
    }

    private static final PoseErrorTolerance CORAL_SCORE_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(1.0), 1.0);
    public static final AutoConstraintOptions CORAL_SCORE_CONSTRAINTS = new AutoConstraintOptions(5.0, 360, 2.0, 720);

    private static final PoseErrorTolerance INITIAL_DRIVE_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(15.0), 10.0);
    public static final AutoConstraintOptions INITIAL_DRIVE_CONSTRAINTS = new AutoConstraintOptions(5.0, 360, 3.0, 720);

    private final Transform2d INITIAL_DRIVE_OFFSET_LEFT = new Transform2d(0.0, -0.5, Rotation2d.kZero);
    private final Transform2d INITIAL_DRIVE_OFFSET_RIGHT = new Transform2d(0.0, 0.5, Rotation2d.kZero);
    private final Transform2d AWAIT_ARM_UP_LEFT_OFFSET = new Transform2d(0.0, -0.2, Rotation2d.kZero);
    private final Transform2d AWAIT_ARM_UP_RIGHT_OFFSET = new Transform2d(0.0, 0.2, Rotation2d.kZero);

    public Command teleopReefAlignAndScore(BooleanSupplier backupDriveInterrupt) {
        return Commands.parallel(
                        requestManager.prepareCoralScoreAndAwaitReady(),
                        trailblazer.followSegment(
                                        new AutoSegment(
                                                INITIAL_DRIVE_CONSTRAINTS,
                                                INITIAL_DRIVE_TOLERANCE,
                                                new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                                    case LEFT ->
                                                            AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_LEFT);
                                                    case RIGHT ->
                                                            AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_RIGHT);
                                                })
                                        )
                                ).onlyIf(() -> AutoAlign.getInstance().approximateDistanceToReef() > 0.5)
                                .andThen(
                                        trailblazer.followSegment(
                                                new AutoSegment(
                                                        CORAL_SCORE_CONSTRAINTS,
                                                        CORAL_SCORE_TOLERANCE,
                                                        new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                                            case LEFT ->
                                                                    AutoAlign.getInstance().usedScoringPose.transformBy(AWAIT_ARM_UP_LEFT_OFFSET);
                                                            case RIGHT ->
                                                                    AutoAlign.getInstance().usedScoringPose.transformBy(AWAIT_ARM_UP_RIGHT_OFFSET);
                                                        })
                                                )
                                        ).until(requestManager::isArmReadyToScoreCoral)
                                )
                ).andThen(
                        trailblazer.followSegment(
                                new AutoSegment(
                                        CORAL_SCORE_CONSTRAINTS,
                                        CORAL_SCORE_TOLERANCE,
                                        new AutoPoint(() -> AutoAlign.getInstance().usedScoringPose)
                                )
                        ),
                        requestManager.executeCoralScoreAndAwaitComplete(),
                        trailblazer.followSegment(
                                new AutoSegment(
                                        CORAL_SCORE_CONSTRAINTS,
                                        CORAL_SCORE_TOLERANCE,
                                        new AutoPoint(() -> switch (AutoAlign.getScoringSideFromRobotPose(AutoAlign.getInstance().usedScoringPose)) {
                                            case LEFT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_LEFT);
                                            case RIGHT ->
                                                    AutoAlign.getInstance().usedScoringPose.transformBy(INITIAL_DRIVE_OFFSET_RIGHT);
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
