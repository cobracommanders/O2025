package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
    public static final AutoConstraintOptions maximumConstraints = new AutoConstraintOptions(
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

//        DogLog.log("1ATesting/LollipopPoseBase", getLollipopApproachPose(Lollipop.LEFT.index));
//        DogLog.log("1ATesting/LollipopPoseTransformed", getLollipopApproachPose(Lollipop.LEFT.index).transformBy(redInitialLollipopOffset));
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

    public Command initialDriveToReefBackNonProcessor() {
        return trailblazer.followSegment(
                new AutoSegment(
                        new AutoConstraintOptions(
                                4.75,
                                Units.degreesToRadians(360.0),
                                6,
                                Units.degreesToRadians(360.0)
                        ),
                        new PoseErrorTolerance(Units.inchesToMeters(8.0), 10.0),
                        new AutoPoint(() -> {
                            Pose2d initial_waypoint = new Pose2d(13, 2.0, Rotation2d.fromDegrees(70.0));
                            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? MathHelpers.pathflip(initial_waypoint) : initial_waypoint;
                        }),
                        new AutoPoint(
                                () -> {
                                    Pose2d final_waypoint = new Pose2d(15.0, 3.5, Rotation2d.fromDegrees(90.0));
                                    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? MathHelpers.pathflip(final_waypoint) : final_waypoint;
                                },
                                requestManager.prepareCoralScoreAndAwaitReady(),
                                maximumConstraints.withMaxLinearAcceleration(3.5)
                        )
                ));
    }

    public Command initialDriveToReefBackProcessor() {
        return trailblazer.followSegment(
                new AutoSegment(
                        new AutoConstraintOptions(
                                4.75,
                                Units.degreesToRadians(360.0),
                                6,
                                Units.degreesToRadians(360.0)
                        ),
                        new PoseErrorTolerance(Units.inchesToMeters(8.0), 10.0),
                        new AutoPoint(() -> {
                            Pose2d initial_waypoint = new Pose2d(
                                    13.0,
                                    (Units.feetToMeters(26) + Units.inchesToMeters(5.0)) - 2.0,
                                    Rotation2d.fromDegrees(70.0)
                            );
                            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? MathHelpers.pathflip(initial_waypoint) : initial_waypoint;
                        }),
                        new AutoPoint(
                                () -> {
                                    Pose2d final_waypoint = new Pose2d(
                                            15.0,
                                            (Units.feetToMeters(26) + Units.inchesToMeters(5.0)) - 3.5,
                                            Rotation2d.fromDegrees(90.0)
                                    );
                                    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ? MathHelpers.pathflip(final_waypoint) : final_waypoint;
                                },
                                requestManager.prepareCoralScoreAndAwaitReady(),
                                maximumConstraints.withMaxLinearAcceleration(3.5)
                        )
                ));
    }

    private final Transform2d redInitialLollipopOffset = new Transform2d(
            Units.inchesToMeters(2.0),
            0.0,
            Rotation2d.kZero
    );

    public Command approachLollipop(Lollipop lollipop) {
        return trailblazer.followSegment(
                new AutoSegment(
                        maximumConstraints.withMaxLinearAcceleration(3.5),
                        new PoseErrorTolerance(Units.inchesToMeters(2), 1.0),
                        new AutoPoint(() -> {
                            Pose2d lollipopApproachPose = getLollipopApproachPose(lollipop.index);
//                            if (
//                                    lollipop == Lollipop.LEFT &&
//                                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
//                            ) {
//                                return lollipopApproachPose.transformBy(redInitialLollipopOffset);
//                            }
                            return lollipopApproachPose;
                        })
                ),
                false); // TODO Change to rotate immediately
    }

    public Command intakeLollipop(Lollipop lollipop) {
        return trailblazer.followSegment(
                new AutoSegment(
                        maximumConstraints.withMaxLinearAcceleration(4.0),
                        new PoseErrorTolerance(Units.inchesToMeters(3), 2.0), // TODO does tolerance matter here? it's not trying to get to a specific point like for scoring, it either picks it up or it doesn't
                        new AutoPoint(() -> {
                            Pose2d lollipopIntakePose = getLollipopIntakePose(lollipop.index);
//                            if (
//                                    lollipop == Lollipop.LEFT &&
//                                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
//                            ) {
//                                return lollipopIntakePose.transformBy(redInitialLollipopOffset);
//                            }
                            return lollipopIntakePose;
                        })),
                false // TODO Change to rotate immediately

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
