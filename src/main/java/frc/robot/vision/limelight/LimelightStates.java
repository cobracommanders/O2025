package frc.robot.vision.limelight;

public enum LimelightStates {
    OFF(0),
    TAGS(0),
    CLOSEST_REEF_TAG(0);

    final int pipelineIndex;

    LimelightStates(int index) {
        this.pipelineIndex = index;
    }
}
