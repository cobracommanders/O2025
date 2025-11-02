package frc.robot.autoAlign;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.autoAlign.tagAlign.TagAlign;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.MathHelpers;

public class AutoAlign extends StateMachine<AutoAlignState> {
    private static final Translation2d CENTER_OF_REEF_RED =
            new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
    private static final Translation2d CENTER_OF_REEF_BLUE =
            new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.5));

    public static RobotScoringSide getNetScoringSideFromRobotPose(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double theta = MathHelpers.angleModulus(robotPose.getRotation().getDegrees());

        // entire field length is 17.55m
        double halfFieldLength = 17.55 / 2.0;

        // Robot is on blue side
        if (robotX < halfFieldLength) {
            if (theta > 0.0) {
                return RobotScoringSide.RIGHT;
            }
            return RobotScoringSide.LEFT;
        }

        // Robot is on red side
        if (theta > 0.0) {
            return RobotScoringSide.LEFT;
        }
        return RobotScoringSide.RIGHT;
    }

    public static Translation2d getAllianceCenterOfReef(boolean isRedAlliance) {
        return isRedAlliance ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
    }

    public static Translation2d getAllianceCenterOfReef(Pose2d robotPose) {
        return robotPose.getX() > 17.55 / 2 ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
    }

    public static Rotation2d angleToReef(Translation2d robotPose, boolean isRed) {
        var centerOfReef = getAllianceCenterOfReef(isRed);
        return Rotation2d.fromRadians(MathUtil.angleModulus(Math.atan2(centerOfReef.getY() - robotPose.getY(), centerOfReef.getX() - robotPose.getX())));
    }

    public static RobotScoringSide getScoringSideFromRobotPose(
            Pose2d robotPose) {
        var centerOfReef = getAllianceCenterOfReef(robotPose.getX() > (17.5 / 2));
        var angleToAim = MathUtil.angleModulus(Math.atan2(centerOfReef.getY() - robotPose.getY(), centerOfReef.getX() - robotPose.getX()));
        var errorRight = Math.abs(MathUtil.angleModulus(angleToAim - (robotPose.getRotation().getRadians() - (Math.PI / 2.0))));
        var errorLeft = Math.abs(MathUtil.angleModulus(angleToAim - (robotPose.getRotation().getRadians() + (Math.PI / 2.0))));

        if (errorRight < errorLeft) {
            return RobotScoringSide.RIGHT;
        }
        return RobotScoringSide.LEFT;
    }

    private final LocalizationSubsystem localization;
    private final TagAlign tagAlign;

    private Pose2d robotPose = Pose2d.kZero;

    private final RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
    private ReefPipe autoalignPipe = ReefPipe.PIPE_A;
    private Pose2d usedScoringPose = Pose2d.kZero;
    private ReefSide bestAlgaeSide = ReefSide.SIDE_AB;

    private AutoAlign() {
        super(AutoAlignState.DEFAULT_STATE, "AutoAlign");
        this.localization = LocalizationSubsystem.getInstance();
        this.tagAlign = new TagAlign(localization);
    }

    public ReefSide getClosestReefSide() {
        return ReefSide.fromPipe(autoalignPipe);
    }

    public Pose2d getAlgaeDistance() {
        return bestAlgaeSide.getPose(ReefSideOffset.BASE, robotScoringSide, robotPose);
    }

    double reefMaxRadius = Units.inchesToMeters(76.0 / 2);

    public double approximateDistanceToReef() {
        Pose2d robotPose = LocalizationSubsystem.getInstance().getPose();
        return getAllianceCenterOfReef(robotPose).getDistance(robotPose.getTranslation())
                - Units.inchesToMeters(35.0 / 2.0) // Distance from center to edge of robot
                - reefMaxRadius; // Distance from center to edge of reef
    }

    @Override
    protected void collectInputs() {
        robotPose = localization.getPose();
        autoalignPipe = tagAlign.getAutoalignPipe();
        usedScoringPose = tagAlign.getUsedScoringPose(autoalignPipe);
        bestAlgaeSide = tagAlign.getAutoalignSide();
        tagAlign.setSide(getScoringSideFromRobotPose(robotPose));
    }

    @Override
    public void periodic() {
        super.periodic();
//        DogLog.log("AutoAlign/UsedScoringPose", usedScoringPose);
//        DogLog.log("AutoAlign/ApproximateDistanceToReef", approximateDistanceToReef());
    }

    public Pose2d getUsedScoringPose() {
        return usedScoringPose;
    }

    private static AutoAlign instance;

    public static AutoAlign getInstance() {
        if (instance == null)
            instance = new AutoAlign(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
