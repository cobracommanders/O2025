package frc.robot;

public final class Constants {
  public static final class ArmConstants {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double G = 0;
      public static final double MotionMagicCruiseVelocity = 0;
      public static final double MotionMagicAcceleration = 0;
      public static final double MotionMagicJerk = 0;
      public static final double encoderOffset = 0;
  }

  public static final class IntakePivotConstants{
    public static final double positionIncrement = 0;
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double G = 0.0;
    public static final double MotionMagicAcceleration = 0.0;
    public static final double MotionMagicCruiseVelocity = 0.0;
    public static final double MotionMagicJerk = 0.0;
  }

  public static final class IntakeRollersConstants{
    public static final double stallCurrent = 0.0;
  }
  public static final class DrivertrainConstants {
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;
  }
  public static class ClimberConstants{
    //TODO: Find the values for these.
    public static final double DEPLOY_MOTION_MAGIC_CRUISE_VELOCITY = 0;
    public static final double DEPLOY_MOTION_MAGIC_ACCELERATION = 0;
    public static final double DEPLOY_MOTION_MAGIC_JERK = 0;
    public static final double CAGE_DETECECTION_CURRENT = 0;
  }
  public static final class HandConstants {
    public static final double coralStallCurrent = 0;
    public static final double algaeStallCurrent = 0;
  }
}
