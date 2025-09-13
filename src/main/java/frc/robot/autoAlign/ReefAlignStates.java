package frc.robot.autoAlign;

public enum ReefAlignStates {
  ALL_CAMERAS_DEAD(false),

  // Checking based on tags
  NO_TAGS_WRONG_POSITION(false),
  NO_TAGS_IN_POSITION(true),
  HAS_TAGS_WRONG_POSITION(false),
  HAS_TAGS_IN_POSITION(true);

  public final boolean aligned;

  private ReefAlignStates(boolean aligned) {
    this.aligned = aligned;
  }
}