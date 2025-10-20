package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.FieldConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
    private final AutoConstraintOptions maximumConstraints = new AutoConstraintOptions(
            4.75, // Linear Velocity
            Units.degreesToRadians(360.0), // Angular Velocity
            6.0, // Linear Acceleration
            Units.degreesToRadians(720.0) // Angular Acceleration
    );

    private final RequestManager requestManager;
    private final Trailblazer trailblazer;

    public AutoBlocks(RequestManager requestManager, Trailblazer trailblazer) {
        this.requestManager = requestManager;
        this.trailblazer = trailblazer;

        DogLog.log("AlgaePositions", new Pose2d[]{
                getLollipopIntakePose(0),
                getLollipopIntakePose(1),
                getLollipopIntakePose(2),
        });
    }

    public static final Transform2d LOLLIPOP_INTAKE_OFFSET = new Transform2d(
            (DrivetrainConstants.WIDTH_METERS / 2) + Units.inchesToMeters(2.0),
            Units.inchesToMeters(ArmConstants.inchesFromCenter),
            Rotation2d.fromDegrees(-90));

    private Pose2d getLollipopIntakePose(int lollipop) {
        Translation2d translation = FieldConstants.StagingPositions.iceCreams[lollipop];
        Pose2d blue = new Pose2d(translation, AutoAlign.angleToReef(translation, false));
        blue = blue.plus(LOLLIPOP_INTAKE_OFFSET);

        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(blue) : blue;
    }

    private static final Transform2d APPROACH_LOLLIPOP_OFFSET = new Transform2d(
            0,
            Units.inchesToMeters(
                    FieldConstants.kAlgaeRadiusInches + // Account for algae diameter
                            5.0 + // Account for hand height out of the frame
                            4.0 // Now actually leave some space
            ),
            Rotation2d.kZero
    );

    private Pose2d getLollipopApproachPose(int lollipop) {
        return getLollipopIntakePose(lollipop).plus(APPROACH_LOLLIPOP_OFFSET);
    }

    public Command driveToBackReefRedNonProcessor() {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                new AutoConstraintOptions(
                                        4.0,
                                        Units.degreesToRadians(360.0),
                                        6.0,
                                        Units.degreesToRadians(360.0)
                                ),
                                new PoseErrorTolerance(Units.inchesToMeters(6.0), 10.0),
                                new AutoPoint(new Pose2d(14.75, 2.0, Rotation2d.fromDegrees(70.0))),
                                new AutoPoint(
                                        new Pose2d(14.75, 4.0, Rotation2d.fromDegrees(90.0)),
                                        requestManager.prepareCoralScoreAndAwaitReady().asProxy(),
                                        maximumConstraints.withMaxLinearAcceleration(3.0)
                                )
                        ))
        );
    }

    public Command approachLollipop(Lollipop lollipop) {
        return trailblazer.followSegment(
                new AutoSegment(
                        maximumConstraints.withMaxLinearAcceleration(4.0),
                        new PoseErrorTolerance(Units.inchesToMeters(0.75), 1.0),
                        new AutoPoint(() -> getLollipopApproachPose(lollipop.index))
                ));
    }

    public Command intakeLollipop(Lollipop lollipop) {
        return trailblazer.followSegment(
                new AutoSegment(
                        maximumConstraints.withMaxLinearAcceleration(2.0),
                        new PoseErrorTolerance(Units.inchesToMeters(0.75), 1.0), // TODO does tolerance matter here? it's not trying to get to a specific point like for scoring, it either picks it up or it doesn't
                        new AutoPoint(() -> getLollipopIntakePose(lollipop.index)))
        );
    }

    public enum Lollipop {
        RIGHT(0),
        MIDDLE(1),
        LEFT(2);

        public final int index;

        Lollipop(int val) {
            this.index = val;
        }
    }
}
