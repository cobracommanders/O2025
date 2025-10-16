package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.FieldConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.AutoPoint;
import frc.robot.trailblazer.AutoSegment;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
    private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE = new PoseErrorTolerance(0.6, 25);

        private static final PoseErrorTolerance LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.6, 10);

        private static final PoseErrorTolerance SUPER_FAST_LOLLIPOP_APPROACH_TOLERANCE = new PoseErrorTolerance(0.8,
                        20);
        public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.4, 10);

        private static final PoseErrorTolerance DEFAULT_POSITION_TOLERANCE =
            new PoseErrorTolerance(0.1, 5);

        public static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET = new Transform2d(-0.6, -0.9,
                        Rotation2d.kZero);

        private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET = new Transform2d(0,
                        Units.inchesToMeters(-60),
                        Rotation2d.kZero);

        private static final Transform2d APPROACH_LOLLIPOP_OFFSET = new Transform2d(0, Units.inchesToMeters(15),
                        Rotation2d.kZero);

        public Transform2d clearReefOffset = new Transform2d(new Translation2d(0.4, -1), Rotation2d.kZero);

        public static final Transform2d RIGHT_LOLLIPOP_OFFSET = new Transform2d(
                        0.0,
                        Units.inchesToMeters(ArmConstants.inchesFromCenter - 4.5),
                        Rotation2d.fromDegrees(-90));
        public static final Transform2d LEFT_LOLLIPOP_OFFSET = new Transform2d(
                        0.0,
                        Units.inchesToMeters(ArmConstants.inchesFromCenter),
                        Rotation2d.fromDegrees(-90));
        public static final Transform2d CENTER_LOLLIPOP_OFFSET = new Transform2d(
                        0.0,
                        Units.inchesToMeters(ArmConstants.inchesFromCenter - 2),
                        Rotation2d.fromDegrees(-90));
        public static final AutoConstraintOptions MAX_CONSTRAINTS = new AutoConstraintOptions(4.75, 57, 4.0, 30);
        public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS = MAX_CONSTRAINTS.withMaxLinearVelocity(5)
                        .withMaxLinearAcceleration(4.5);
        public static final AutoConstraintOptions BASE_CONSTRAINTS = new AutoConstraintOptions(4.75, 30, 1.25, 25);

        public static final AutoConstraintOptions CORAL_MAP_CONSTRAINTS = new AutoConstraintOptions(4.0, 10, 2.5, 10);
        private static final AutoConstraintOptions SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(2);
                        //.withMaxLinearAcceleration(1.75);
        private static final AutoConstraintOptions L2_SCORING_CONSTRAINTS = BASE_CONSTRAINTS.withMaxLinearVelocity(3.3)
                        .withMaxLinearAcceleration(2.15);
        private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS;
                        // .withMaxLinearAcceleration(2.0)
                        // .withMaxLinearVelocity(1.7).withMaxAngularVelocity(40);

        private static final AutoConstraintOptions SUPER_FAST_LOLLIPOP_CONSTRAINTS = BASE_CONSTRAINTS
                        .withMaxLinearAcceleration(3.0).withMaxLinearVelocity(4.5);

        public static final AutoConstraintOptions BASE_CONSTRAINTS_FOR_GROUND_AUTOS = new AutoConstraintOptions(3.75,
                        57,
                        1.75, 25);
        private static final AutoConstraintOptions SCORING_CONSTRAINTS_FOR_GROUND_AUTOS = BASE_CONSTRAINTS_FOR_GROUND_AUTOS
                        .withMaxLinearAcceleration(1.25).withMaxLinearVelocity(3);

        private final RequestManager requestManager;
        private final Trailblazer trailblazer;

    public AutoBlocks(RequestManager requestManager, Trailblazer trailblazer) {
        this.requestManager = requestManager;
        this.trailblazer = trailblazer;
    }

    private Pose2d getLollipopCoordsBlue(int lollipop) {
        if (Lollipop.MIDDLE.index == lollipop) {
                        Translation2d translation = FieldConstants.StagingPositions.iceCreams[lollipop];
                        Pose2d blue = new Pose2d(translation, AutoAlign.angleToReef(translation, false));
                        blue = blue.plus(CENTER_LOLLIPOP_OFFSET);
                        return blue;
                }
                // else if (Lollipop.LEFT.index == lollipop) {
                //         Translation2d translation = FieldConstants.StagingPositions.iceCreams[lollipop];
                //         Pose2d blue = new Pose2d(translation, AutoAlign.angleToReef(translation, false));
                //         blue = blue.plus(LEFT_LOLLIPOP_OFFSET);
                //         return blue;
                // }
                Translation2d translation = FieldConstants.StagingPositions.iceCreams[lollipop];
                Pose2d blue = new Pose2d(translation, AutoAlign.angleToReef(translation, false));
                blue = blue.plus(RIGHT_LOLLIPOP_OFFSET);
        return blue;
    }

    private Pose2d getLolliApproach(int lollipop) {
        Pose2d blue = getLollipopCoordsBlue(lollipop).plus(APPROACH_LOLLIPOP_OFFSET);
        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(blue) : blue;
    }

    private Pose2d getLolliIntake(int lollipop) {
        Pose2d blue = getLollipopCoordsBlue(lollipop);//.plus(LOLLIPOP_OFFSET);
        return FmsSubsystem.getInstance().isRedAlliance() ? MathHelpers.pathflip(blue) : blue;
    }

        public Pose2d getClearReefOffsetPose(ReefPipe pipe, RobotScoringSide scoringSide) {
                return pipe.getPose(ReefPipeLevel.L4, scoringSide).transformBy(clearReefOffset);
        }
        public Command scoreL2(ReefPipe pipe, RobotScoringSide scoringSide) {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L2,
                                                                                scoringSide)))),
//                                RobotCommands.getInstance().autoReefAlignCommand(),
                                //Robot.robotCommands.waitForWaitL4(),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                                BASE_CONSTRAINTS,
                                                                AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L2, scoringSide))))
                                        // Moves the arm in parallel, the group won't move on until this command finishes and the arm is ready
                                        .alongWith(requestManager.prepareCoralScoreAndAwaitReady(FieldConstants.PipeScoringLevel.L2)),

                                requestManager.executeCoralScoreAndAwaitComplete());
        }


        public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide) {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4,
                                                                                scoringSide)))),
//                                RobotCommands.getInstance().autoReefAlignCommand(),
                                //Robot.robotCommands.waitForWaitL4(),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                                BASE_CONSTRAINTS,
                                                                AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4,
                                                                                scoringSide))))
                                        // Moves the arm in parallel, the group won't move on until this command finishes and the arm is ready
                                        .alongWith(requestManager.prepareCoralScoreAndAwaitReady(FieldConstants.PipeScoringLevel.L4))
                        ,
                                requestManager.executeCoralScoreAndAwaitComplete());
        }

        public Command scorePreloadL4(ReefPipe pipe, RobotScoringSide scoringSide) {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4,
                                                                                scoringSide)))),
                                //Robot.robotCommands.waitForWaitL4(),
//                                RobotCommands.getInstance().autoReefAlignCommand(),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                                BASE_CONSTRAINTS,
                                                                DEFAULT_POSITION_TOLERANCE,
                                                                //AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L4,
                                                                                scoringSide))))
                        // Moves the arm in parallel, the group won't move on until this command finishes and the arm is ready
                                        .alongWith(requestManager.prepareCoralScoreAndAwaitReady(FieldConstants.PipeScoringLevel.L4))
                        ,
                                requestManager.executeCoralScoreAndAwaitComplete());
        }

        public Command driveToBackReefRedNonProcessor() {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                        new AutoPoint(new Pose2d(12.836, 1.085, Rotation2d.fromDegrees(90.0))),
                                                        new AutoPoint(new Pose2d(14.644, 2.240, Rotation2d.fromDegrees(90.0))))));
        }

        public Command driveToBackReefRedProcessor() {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                        new AutoPoint(new Pose2d(12.836, 6.965, Rotation2d.fromDegrees(90))),
                                                        new AutoPoint(new Pose2d(14.644, 5.81, Rotation2d.fromDegrees(90))))));
        }

        public Command driveToBackReefBlueNonProcessor() {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                        new AutoPoint(new Pose2d(4.714, 6.965, Rotation2d.fromDegrees(-90))),
                                                        new AutoPoint(new Pose2d(2.906, 5.81, Rotation2d.fromDegrees(-90))))));
        }

        public Command driveToBackReefBlueProcessor() {
                return Commands.sequence(
                                //new WaitCommand(0.2),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                        BASE_CONSTRAINTS,
                                                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                                                        new AutoPoint(new Pose2d(4.714, 1.085, Rotation2d.fromDegrees(-90))),
                                                        new AutoPoint(new Pose2d(2.906, 2.24, Rotation2d.fromDegrees(-90))))));
        }

        public Command backUpFromReef(ReefPipe pipe, RobotScoringSide scoringSide) {
                return trailblazer.followSegment(
                                new AutoSegment(
                                                BASE_CONSTRAINTS,
                                                AFTER_SCORE_POSITION_TOLERANCE,
                                                new AutoPoint(() -> getClearReefOffsetPose(pipe, scoringSide))));
        }

        public Command pickUpLolli(Lollipop lollipop, ReefPipe pipe, RobotScoringSide scoringSide) {
                return Commands.sequence(
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                                BASE_CONSTRAINTS,
                                                                AutoBlocks.LOLLIPOP_APPROACH_TOLERANCE,
                                                                new AutoPoint(() -> getLolliApproach(lollipop.index))

                                                )),
                                trailblazer.followSegment(
                                                new AutoSegment(
                                                                LOLLIPOP_CONSTRAINTS,
                                                                DEFAULT_POSITION_TOLERANCE,
                                                                new AutoPoint(() -> getLolliIntake(lollipop.index)))));
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
