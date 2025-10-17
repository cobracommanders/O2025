package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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

public class RobotCommands {
    private final Trailblazer trailblazer;
    private final RequestManager requestManager;

    public RobotCommands(Trailblazer trailblazer, RequestManager requestManager) {
        this.requestManager = requestManager;
        this.trailblazer = trailblazer;
    }


    /* ******** AUTO SCORE ******** */


    // Tight tolerance for scoring coral
    private static final PoseErrorTolerance CORAL_SCORE_TOLERANCE = new PoseErrorTolerance(Units.inchesToMeters(0.75), 1.0);
    // Constraints for driving while mechanisms are extended
    // TODO consider using different constraints for different levels
    private static final AutoConstraintOptions EXTENDED_DRIVE_CONSTRAINTS = new AutoConstraintOptions(5.0, Units.degreesToRadians(360.0), 2.0, Units.degreesToRadians(720.0));

    // Constraints for driving while mechanisms are extended
    // TODO consider using different constraints for different levels
    private static final AutoConstraintOptions SPEED_DRIVE_CONSTRAINTS = new AutoConstraintOptions(5.0, Units.degreesToRadians(360.0), 4.0, Units.degreesToRadians(720.0));

    // Position offsets
    // These are set up as constants because they are called periodically while trailblazer runs, and doing this reduces objects created and therefore reduces garbage collection time

    // Offset that is used to wait for the arm and elevator to be in position to avoid hitting the reef
    // Just far enough back for the mechanisms to move freely
    private final Transform2d AWAIT_ARM_LEFT_OFFSET = new Transform2d(0.0, -0.2, Rotation2d.kZero);
    private final Transform2d AWAIT_ARM_RIGHT_OFFSET = new Transform2d(0.0, 0.2, Rotation2d.kZero);

    // Distance to drive back after scoring to pull the coral out of the hand and signal to the driver that the sequence is complete
    // Kind of arbitrary, and it is interrupted when the driver touches the controls
    private final Transform2d DRIVE_BACK_AFTER_SCORE_LEFT_OFFSET = new Transform2d(0.0, -0.5, Rotation2d.kZero);
    private final Transform2d DRIVE_BACK_AFTER_SCORE_RIGHT_OFFSET = new Transform2d(0.0, 0.5, Rotation2d.kZero);


    // .asProxy() means that the full command (.teleopReefAlignAndScore) won't require the subsystems used
    // by the child command (eg .prepareCoralScoreAndAwaitReady) decorated with .asProxy() until it actually
    // runs that command. This is used to start driving while allowing another command that is using the
    // armManager (like the handoff) to still run without being interrupted.
    //
    // Note that this can be somewhat complicated, refer to the warnings in the wpilib docs before using it if possible.
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html#scheduling-other-commands


    // 1. Prepare the arm for scoring while driving close to the reef
    // 2. Drive up to the final scoring position
    // 3. Score coral
    // 4. Drive backwards to pull the coral out and signal that the sequence is complete
    public Command teleopReefAlignAndScore(BooleanSupplier backupDriveInterrupt) {
        return sequence(
                // Start by driving up to the reef and preparing the arm for scoring in parallel
                parallel(
                        sequence(
                                // Wait until the hand has a coral
                                // This lets the command run while the handoff is happening without causing issues
                                waitUntil(() -> requestManager.getHandGamePiece().isCoral() && requestManager.isArmIdle()),

                                // Set arm and elevator to prepare score state
                                requestManager.prepareCoralScoreAndAwaitReady().asProxy() // See note above for .asProxy()
                        ),
                        // Drive up to the AWAIT_ARM_OFFSET position
                        // This ensures the robot doesn't get too close to the reef while the arm is still preparing
                        trailblazer.followSegment(new AutoSegment(EXTENDED_DRIVE_CONSTRAINTS, CORAL_SCORE_TOLERANCE, new AutoPoint(() -> {
                                    // Switch between the offsets based on the side the robot is scoring on
                                    Pose2d scoringPose = AutoAlign.getInstance().getUsedScoringPose();
                                    return switch (AutoAlign.getScoringSideFromRobotPose(scoringPose)) {
                                        case LEFT -> scoringPose.transformBy(AWAIT_ARM_LEFT_OFFSET);
                                        case RIGHT -> scoringPose.transformBy(AWAIT_ARM_RIGHT_OFFSET);
                                    };
                                })))
                                // .until cancels this drive command once the arm is in the ready state (after it's done preparing)
                                .until(requestManager::isArmReadyToScoreCoral)
                ),

                // Drive to the final scoring position now that the arm is ready to score
                trailblazer.followSegment(new AutoSegment(EXTENDED_DRIVE_CONSTRAINTS, CORAL_SCORE_TOLERANCE, new AutoPoint(() -> {
                    return AutoAlign.getInstance().getUsedScoringPose();
                }))),
                // Once the drive command finishes, score the coral and wait for the arm to finish moving
                requestManager.executeCoralScoreAndAwaitComplete().asProxy(), // See note above for .asProxy()
                // Drive back after scoring to pull the coral out of the hand and signal to the driver that the sequence is complete
                trailblazer.followSegment(new AutoSegment(SPEED_DRIVE_CONSTRAINTS, CORAL_SCORE_TOLERANCE, new AutoPoint(() -> {
                            // Switch between the offsets based on the side the robot is scoring on
                            Pose2d scoringPose = AutoAlign.getInstance().getUsedScoringPose();
                            return switch (AutoAlign.getScoringSideFromRobotPose(scoringPose)) {
                                case LEFT -> scoringPose.transformBy(DRIVE_BACK_AFTER_SCORE_LEFT_OFFSET);
                                case RIGHT -> scoringPose.transformBy(DRIVE_BACK_AFTER_SCORE_RIGHT_OFFSET);
                            };
                        })))
                        // Cancel the command when the interrupt is received
                        .until(backupDriveInterrupt))
                // .finallyDo will be called even if the command is interrupted, so the drivetrain should never be locked out of the proper state
                // .beforeStarting is used because it kind of matches .finallyDo and this lets both state controls be together in the command
                .beforeStarting(() -> DriveSubsystem.getInstance().requestReefAlign())
                .finallyDo(() -> DriveSubsystem.getInstance().requestTeleop());
    }

    /* ******** MISC. ******** */

    public Command algaeAlignCommand() {
        return runOnce(() -> AutoAlign
                .getInstance()
                .setAlgaeIntakingOffset(ReefSideOffset.ALGAE_INTAKING)).andThen(runOnce(() -> DriveSubsystem
                .getInstance()
                .requestAlgaeAlign()));
    }

    public Command driveTeleopCommand() {
        return runOnce(() -> DriveSubsystem.getInstance().requestTeleop());
    }
}
