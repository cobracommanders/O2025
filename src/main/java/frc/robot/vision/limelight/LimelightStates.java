package frc.robot.vision.limelight;

public enum LimelightStates {
    OFF(1),
    TAGS(1),
    CLOSEST_REEF_TAG(1);
  
    final int pipelineIndex;
  
    LimelightStates(int index) {
      this.pipelineIndex = index;
    }
  }