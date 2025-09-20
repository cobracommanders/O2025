package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.commands.RobotCommands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {

    private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE = new PoseErrorTolerance(0.6, 25);

    private static final PoseErrorTolerance LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.6, 10);

    private static final PoseErrorTolerance SUPER_FAST_LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.8, 20);
    public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);

    public static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET = new Transform2d(-0.6, -0.9, Rotation2d.kZero);

    private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET = new Transform2d(0, Units.inchesToMeters(-60),
            Rotation2d.kZero);

    private static final Transform2d CENTER_LOLLIPOP_OFFSET = new Transform2d(0, Units.inchesToMeters(5),
            Rotation2d.kZero);
    private static final Transform2d APPROACH_LOLLIPOP_OFFSET = new Transform2d(0, Units.inchesToMeters(15),
            Rotation2d.kZero);

    public static final Transform2d LOLLIPOP_OFFSET = new Transform2d(
            0.0,
            -Units.inchesToMeters(ArmConstants.inchesFromCenter),
            Rotation2d.fromDegrees(90));
    public static final AutoConstraintOptions MAX_CONSTRAINTS = new AutoConstraintOptions(4.75, 57, 4.0, 30);
    public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS = MAX_CONSTRAINTS.withMaxLinearVelocity(5)
            .withMaxLinearAcceleration(4.5);
    public static final AutoConstraintOptions BASE_CONSTRAINTS = new AutoConstraintOptions(4.0, 30, 2.5, 25);

    public static final AutoConstraintOptions CORAL_MAP_CONSTRAINTS = new AutoConstraintOptions(4.0, 10, 2.5, 10);
    private static final AutoConstraintOptions SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(1.5)
            .withMaxLinearAcceleration(1.75);
    private static final AutoConstraintOptions L2_SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(3.3)
            .withMaxLinearAcceleration(2.15);
    private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0)
            .withMaxLinearVelocity(1.7);

    private static final AutoConstraintOptions SUPER_FAST_LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS
            .withMaxLinearAcceleration(3.0).withMaxLinearVelocity(4.5);

    public static final AutoConstraintOptions BASE_CONSTRAINTS_FOR_GROUND_AUTOS = new AutoConstraintOptions(3.75, 57,
            1.75, 25);
    private static final AutoConstraintOptions SCORING_CONSTRAINTS_FOR_GROUND_AUTOS = BASE_CONSTRAINTS_FOR_GROUND_AUTOS
            .withMaxLinearAcceleration(1.25).withMaxLinearVelocity(3);

    private final Trailblazer trailblazer;

    private final AutoCommands autoCommands = AutoCommands.getInstance();

    public AutoBlocks(Trailblazer trailblazer) {
        this.trailblazer = trailblazer;
    }

    public Transform2d clearReefOffset = new Transform2d(new Translation2d(0, -0.6), Rotation2d.kZero);
    public Pose2d getClearReefOffsetPose(ReefPipe pipe, RobotScoringSide scoringSide) {
        return pipe.getPose(ReefPipeLevel.L4, scoringSide).transformBy(clearReefOffset);
    }
    public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                BASE_CONSTRAINTS,
                                AutoBlocks.APPROACH_REEF_TOLERANCE,
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                                Robot.robotCommands.prepareScoreWithHandoffCheckCommand()))),
                trailblazer.followSegment(
                        new AutoSegment(
                                SCORING_CONSTRAINTS,
                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4, scoringSide),
                                        Robot.robotCommands.autoReefAlignCommand()))
                                ),
                Commands.either(RobotCommands.getInstance().waitForWaitL4(), RobotCommands.getInstance().prepareScoreCommand().andThen(RobotCommands.getInstance().waitForWaitL4()), ()-> ArmManager.getInstance().getState() == ArmManagerStates.PREPARE_SCORE_L4 || ArmManager.getInstance().getState() == ArmManagerStates.WAIT_L4),
                RobotCommands.getInstance().scoreCommand().andThen(ArmManager.getInstance().finishScoring())
                // trailblazer.followSegment(
                //     new AutoSegment(
                //             SCORING_CONSTRAINTS,
                //             new AutoPoint(() -> getClearReefOffsetPose(pipe, scoringSide))))
        );
    }

    private Pose2d getLollipopCoords(int lollipop) {
        Pose2d blue = new Pose2d(FieldConstants.StagingPositions.iceCreams[lollipop], Rotation2d.kZero);
        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(blue) : blue;
    }
    
    private Pose2d lollipopCoords = new Pose2d(FieldConstants.StagingPositions.iceCreams[1], Rotation2d.kZero);
    private Pose2d approachLolliOffset = new Pose2d(new Translation2d(1, 0), Rotation2d.kZero).transformBy(new Transform2d(Translation2d.kZero, Rotation2d.kCW_90deg));
    private Pose2d getMidLolli() {
        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(lollipopCoords) : lollipopCoords;
    }
    private Pose2d getLolliApproach() {
        return getMidLolli().plus(approachLolliOffset.minus(Pose2d.kZero));
    }
    private Pose2d getLolliIntake() {
        return getMidLolli().transformBy(new Transform2d(Translation2d.kZero, Rotation2d.kCW_90deg));
    }

    private Pose2d getLolliApproach(int lollipop) {
        Transform2d offset = new Pose2d(-1, 0, Rotation2d.kCCW_90deg).rotateBy(AutoAlign.angleToReef(getLollipopCoords(lollipop))).minus(Pose2d.kZero);
        return getLollipopCoords(lollipop).plus(offset);
    }
    private Pose2d getLolliIntake(int lollipop) {
        return getLollipopCoords(lollipop).transformBy(new Transform2d(Translation2d.kZero, Rotation2d.kCW_90deg));
    }

    // private Pose2d approachLolli = new Pose2d(getMidLolli()., Rotation2d.fromDegrees());
    public Command pickUpMidLolli(ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                LOLLIPOP_CONSTRAINTS,
                                AutoBlocks.LOLLIPOP_APPROACH_TOLERANCE,
                                new AutoPoint(()-> getClearReefOffsetPose(pipe, scoringSide)),
                                new AutoPoint(()-> getLolliApproach()
                                ),
                                new AutoPoint(()-> getLolliIntake())
                        )
                )
        );
    }
    public Command pickUpRightLolliFromPreload() {
        return Commands.sequence(
                trailblazer.followSegment(
                    new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    LOLLIPOP_APPROACH_TOLERANCE,
                    new AutoPoint(
                        MathHelpers.pathflip(new Pose2d(4, 6, Rotation2d.fromDegrees(-120 - 90)))
                    ))
                ),
                trailblazer.followSegment(
                        new AutoSegment(
                                SUPER_FAST_LOLLIPOP_CONSTRAINTS,
                                AutoBlocks.SUPER_FAST_LOLLIPOP_APPROACH_TOLERANCE,
                                new AutoPoint(()-> getLolliApproach(2)
                                ),
                                new AutoPoint(()-> getLolliIntake(2))
                        )
                )
        );
    }
    public Command pickUpLeftLolli(ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                LOLLIPOP_CONSTRAINTS,
                                AutoBlocks.LOLLIPOP_APPROACH_TOLERANCE,
                                new AutoPoint(()-> getClearReefOffsetPose(pipe, scoringSide)),
                                new AutoPoint(()-> getLolliApproach(0)
                                ),
                                new AutoPoint(()-> getLolliIntake(0))
                        )
                )
        );
    }
    public Command pickUpRightLolli(ReefPipe pipe, RobotScoringSide scoringSide) {
        return Commands.sequence(
                trailblazer.followSegment(
                        new AutoSegment(
                                LOLLIPOP_CONSTRAINTS,
                                AutoBlocks.LOLLIPOP_APPROACH_TOLERANCE,
                                new AutoPoint(()-> getClearReefOffsetPose(pipe, scoringSide)),
                                new AutoPoint(()-> getLolliApproach(2)
                                ),
                                new AutoPoint(()-> getLolliIntake(2))
                        )
                )
        );
    }
}
