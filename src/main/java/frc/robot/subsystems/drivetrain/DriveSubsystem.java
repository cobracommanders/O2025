package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Robot;
import frc.robot.autoAlign.ReefAlignStates;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPoses;
import frc.robot.fms.FmsSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.MathHelpers;

public class DriveSubsystem extends StateMachine<DriveStates> {
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoAlignSpeeds = new ChassisSpeeds();
  public final CommandSwerveDrivetrain drivetrain;
  public Pose2d closestBranch = new Pose2d();
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  public final Pigeon2 drivetrainPigeon = CommandSwerveDrivetrain.getInstance().getPigeon2();
  private double teleopSlowModePercent = 1.0;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;

  private double goalSnapAngle = 0;

  private double elevatorHeight;

  private static final double LEFT_X_DEADBAND = 0.05;
  private static final double LEFT_Y_DEADBAND = 0.05;
  private static final double RIGHT_X_DEADBAND = 0.15;

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE = InterpolatingDoubleTreeMap
      .ofEntries(Map.entry(0.0, 1.0), Map.entry(0.6, 1.0), Map.entry(1.4, 0.7));

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(DrivetrainConstants.maxSpeed * 0.03)
      .withRotationalDeadband(DrivetrainConstants.maxAngularRate * 0.03);

  public final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(DrivetrainConstants.maxSpeed * 0.03)
      .withRotationalDeadband(DrivetrainConstants.maxAngularRate * 0.03);

  public DriveSubsystem() {
    super(DriveStates.TELEOP);
    drivetrain = CommandSwerveDrivetrain.getInstance();
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        () -> drivetrain.getState().Pose, // Robot pose supplier
        drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        () -> drivetrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> setAutoSpeeds(speeds), // Method that will drive the robot given ROBOT
                                                         // RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                        // Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          return FmsSubsystem.getInstance().isRedAlliance();
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public Translation2d getControllerValues() {
    if (getState() != DriveStates.REEF_ALIGN_TELEOP) {
      return Translation2d.kZero;
    }

    return new Translation2d(
        ControllerHelpers.getJoystickMagnitude(
            rawControllerXValue, rawControllerYValue, 2),
        new Rotation2d(rawControllerXValue, rawControllerYValue));
  }

  public void setAutoAlignSpeeds(ChassisSpeeds speeds) {
    autoAlignSpeeds = speeds;
    sendSwerveRequest(DriveStates.REEF_ALIGN_TELEOP);
  }

  public void scoringAlignmentRequest() {
    if (DriverStation.isAutonomous()) {
      normalDriveRequest();
    } else {
      setSnapToAngle(ReefPoses.getInstance().getNearestBranch().getRotation().getDegrees());
      setStateFromRequest(DriveStates.REEF_ALIGN_TELEOP);
    }
  }

  public void normalDriveRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(DriveStates.AUTO);
    } else {
      setStateFromRequest(DriveStates.TELEOP);
    }
  }


  public void setSnapToAngle(double angle) {
    goalSnapAngle = ReefPoses.getInstance().getNearestBranch().getRotation().getDegrees();

    // We don't necessarily set auto swerve speeds every loop, so this ensures we are always snapped
    // to the right angle during auto. Teleop doesn't need this since teleop speeds are constantly
    // fed into swerve.
    if (DriverStation.isAutonomous()) {
      sendSwerveRequest(DriveStates.REEF_ALIGN_TELEOP);
    }
  }

  @Override
  protected DriveStates getNextState(DriveStates currentState) {
    return switch (currentState) {
      case AUTO, TELEOP ->
        FmsSubsystem.getInstance().isAutonomous() ? DriveStates.AUTO : DriveStates.TELEOP;
      case REEF_ALIGN_TELEOP ->
        DriverStation.isAutonomous() ? DriveStates.AUTO : DriveStates.REEF_ALIGN_TELEOP;
    };
  }

  public void setTeleopSpeeds(double x, double y, double theta) {
    rawControllerXValue = x;
    rawControllerYValue = y;
    double leftY = -1.0
        * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(y, LEFT_Y_DEADBAND), 2.0);
    double leftX = -1.0
        * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(x, LEFT_X_DEADBAND), 2.0);
    double rightX = MathHelpers.signedExp(
        ControllerHelpers.deadbandJoystickValue(theta, RIGHT_X_DEADBAND), 1.3);

    DogLog.log("Swerve/LeftX", leftX);
    DogLog.log("Swerve/LeftY", leftY);
    DogLog.log("Swerve/RightX", rightX);
    Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
    double mappedX = mappedpose.getX();
    double mappedY = mappedpose.getY();

    teleopSpeeds = new ChassisSpeeds(
        // TODO if robot is driving backwards, get rid of the invert here
        mappedY * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
        mappedX * DrivetrainConstants.maxSpeed * teleopSlowModePercent,
        rightX * DrivetrainConstants.maxAngularRate * teleopSlowModePercent);
  }

  public ChassisSpeeds getTeleopSpeeds() {
    return teleopSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, drivetrainState.Pose.getRotation());
  }

  @Override
  public void simulationPeriodic() {
    drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
  }

  @Override
  protected void collectInputs() {
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
    DogLog.log("Swerve/SlowModePercent", teleopSlowModePercent);
    DogLog.log("Swerve/Pose", drivetrain.getState().Pose);
  }

  @Override
  public void periodic() {
    super.periodic();
    drivetrain.update();
    sendSwerveRequest(getState());
  }

  public void setAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
  }

  public void setState(DriveStates newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(DriveStates newState) {
    switch (newState) {
      default -> {
      }
    }
  }

  protected void sendSwerveRequest(DriveStates newState) {
    switch (newState) {
      case TELEOP -> {
        drivetrain.setControl(
            drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      case AUTO -> {
        drivetrain.setControl(
            driveRobotRelative
                .withVelocityX(autoSpeeds.vxMetersPerSecond)
                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

      case REEF_ALIGN_TELEOP -> {
        drivetrain.setControl(
            drive
                .withVelocityX(autoAlignSpeeds.vxMetersPerSecond)
                .withVelocityY(autoAlignSpeeds.vyMetersPerSecond)
                .withRotationalRate(autoAlignSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }
    }
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }

  private static DriveSubsystem instance;

  public static DriveSubsystem getInstance() {
    if (instance == null)
      instance = new DriveSubsystem(); // Make sure there is an instance (this will only run once)
    return instance;
  }
}
