package frc.robot.autoAlign.tagAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autoAlign.*;
import frc.robot.localization.LocalizationSubsystem;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class TagAlign {
    public static final List<ReefPipe> ALL_REEF_PIPES =
            List.copyOf(Arrays.asList(ReefPipe.values()));
    public static final List<ReefSide> ALL_REEF_SIDES =
            List.copyOf(Arrays.asList(ReefSide.values()));

    private final LocalizationSubsystem localization;

    private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;

    private final Comparator<ReefPipe> reefPipeSelector;
    private final Comparator<ReefSide> reefSideSelector;

    public TagAlign(LocalizationSubsystem localization) {
        this.localization = localization;

        reefPipeSelector = Comparator.comparingDouble((pipe) -> {
            Translation2d robotPose = localization.getPose().getTranslation();
            Translation2d pipePose = pipe.getPose(ReefPipeLevel.L3, robotScoringSide, localization.getPose()).getTranslation();
            return pipePose.getDistance(robotPose);
        });

        reefSideSelector = Comparator.comparingDouble((side) -> {
            Translation2d robotPose = localization.getPose().getTranslation();
            Translation2d sidePose = side.getPose(ReefSideOffset.ALGAE_INTAKING, robotScoringSide, localization.getPose()).getTranslation();
            return sidePose.getDistance(robotPose);
        });
    }

    public void setSide(RobotScoringSide side) {
        this.robotScoringSide = side;
    }

    public Pose2d getUsedScoringPose(ReefPipe pipe) {
        return pipe.getPose(ReefPipeLevel.L3, robotScoringSide, localization.getPose());
    }

    public ReefPipe getAutoalignPipe() {
        return ALL_REEF_PIPES.stream().min(reefPipeSelector).orElseThrow();
    }

    public ReefSide getAutoalignSide() {
        return ALL_REEF_SIDES.stream().min(reefSideSelector).orElseThrow();
    }
}