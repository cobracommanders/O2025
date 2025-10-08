package frc.robot.vision;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightStates;
import frc.robot.vision.results.OptionalTagResult;

public class VisionSubsystem extends StateMachine<VisionStates> {
  private static final double REEF_CLOSEUP_DISTANCE = 0.7;
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);
  private final Limelight leftBackLimelight;
  private final Limelight leftFrontLimelight;
  private final Limelight rightLimelight;

  private OptionalTagResult leftBackTagResult = new OptionalTagResult();
  private OptionalTagResult leftFrontTagResult = new OptionalTagResult();
  private OptionalTagResult rightTagResult = new OptionalTagResult();
  private OptionalTagResult gamePieceTagResult = new OptionalTagResult();

  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public VisionSubsystem(
      Limelight bl,
      Limelight fl,
      Limelight right) {
    super(VisionStates.TAGS);
    this.leftBackLimelight = bl;
    this.leftFrontLimelight = fl;
    this.rightLimelight = right;
  }

  @Override
  protected void collectInputs() {
//    angularVelocity = Units.radiansToDegrees(DriveSubsystem.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond);
    angularVelocity = DriveSubsystem.getInstance().getAngularVelocityYWorld();

    leftBackTagResult = leftBackLimelight.getTagResult();
    leftFrontTagResult = leftFrontLimelight.getTagResult();
    rightTagResult = rightLimelight.getTagResult();

    if (leftBackTagResult.isPresent()
        || leftFrontTagResult.isPresent()
        || rightTagResult.isPresent()
        || gamePieceTagResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (DriverStation.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getLeftBackTagResult() {
    return leftBackTagResult;
  }


  public OptionalTagResult getLeftFrontTagResult() {
    return leftFrontTagResult;
  }

  public OptionalTagResult getRightTagResult() {
    return rightTagResult;
  }

  public OptionalTagResult getGamePieceTagResult() {
    if (leftBackTagResult.isEmpty() && rightTagResult.isEmpty() && leftFrontTagResult.isEmpty()) {

      return gamePieceTagResult;
    }
    return gamePieceTagResult.empty();
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced;
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionStates state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionStates newState) {
    switch (newState) {
      case TAGS -> {
        leftBackLimelight.setState(LimelightStates.TAGS);
        leftFrontLimelight.setState(LimelightStates.TAGS);
        rightLimelight.setState(LimelightStates.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        if (FeatureFlags.USE_ANY_REEF_TAG.getAsBoolean()) {
          leftBackLimelight.setState(LimelightStates.TAGS);
          leftFrontLimelight.setState(LimelightStates.TAGS);
          rightLimelight.setState(LimelightStates.TAGS);
        } else {
          leftBackLimelight.setState(LimelightStates.CLOSEST_REEF_TAG);
          leftFrontLimelight.setState(LimelightStates.CLOSEST_REEF_TAG);
          rightLimelight.setState(LimelightStates.CLOSEST_REEF_TAG);
        }
      }
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    leftBackLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);
    leftFrontLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);
    rightLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);

    if (FeatureFlags.CAMERA_POSITION_CALIBRATION.getAsBoolean()) {
      setStateFromRequest(VisionStates.TAGS);
      leftBackLimelight.logCameraPositionCalibrationValues();
      leftFrontLimelight.logCameraPositionCalibrationValues();
      rightLimelight.logCameraPositionCalibrationValues();
    }

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);
  }

  public void setClosestScoringReefAndPipe(int tagID) {
    leftFrontLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
    leftBackLimelight.setClosestScoringReefTag(tagID);
  }

  private static VisionSubsystem instance;

      public static VisionSubsystem getInstance() {
    if (instance == null)
      instance = new VisionSubsystem(
        new Limelight("bl", LimelightStates.CLOSEST_REEF_TAG, true)
      , new Limelight("fl", LimelightStates.CLOSEST_REEF_TAG, true)
      , new Limelight("right", LimelightStates.CLOSEST_REEF_TAG, true)); // Make sure there is an instance (this will only run once)
    return instance;
  }
}