package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.armManager.arm.Arm;
import frc.robot.subsystems.armManager.hand.Hand;

public final class MechanismVisualizer {
  private static final Translation2d MECHANISM_AREA =
      new Translation2d(
          ((ArmConstants.armLength + Units.inchesToMeters(2)) * 2.0),
          (ElevatorConstants.maxHeight + Units.inchesToMeters(2))
              + ArmConstants.armLength);
  private static final Mechanism2d mechanism =
      new Mechanism2d(
          MECHANISM_AREA.getX(), MECHANISM_AREA.getY(), new Color8Bit(new Color("#121212")));
  private static final MechanismRoot2d root =
      mechanism.getRoot("superstructure", MECHANISM_AREA.getX() / 2.0, Units.inchesToMeters(2));
  private static final MechanismLigament2d elevator =
      root.append(
          new MechanismLigament2d(
              "elevator",
              Units.inchesToMeters(ElevatorConstants.minHeight),
              90,
              20,
              new Color8Bit(Color.kFirstBlue)));
  private static final MechanismLigament2d arm =
      elevator.append(
          new MechanismLigament2d(
              "arm", ArmConstants.armLength, 90, 10, new Color8Bit(Color.kFirstRed)));

  public static void logArm(double armPosition) {
    // if (!RobotConfig.IS_DEVELOPMENT) {
    //   return;
    // }

    SmartDashboard.putData("SuperstructureVisualization", mechanism);
    var armAngle = -1.0 * (armPosition * 360 + 360 - elevator.getAngle());

    var carriagePose = new Pose3d(new Translation3d(0, 0, elevator.getLength()), Rotation3d.kZero);

    var elevatorPose = new Pose3d(Translation3d.kZero, Rotation3d.kZero);
    var armPose =
        new Pose3d(
            new Translation3d(0, 0, elevator.getLength()),
            new Rotation3d(Units.degreesToRadians(armAngle), 0, 0));
    DogLog.log(
        "SuperstructureVisualization/SuperStructure3d",
        new Pose3d[] {elevatorPose,carriagePose, armPose});
    arm.setAngle(armAngle);
  }

  public static void logElevator(double elevatorPosition) {
    // if (!RobotConfig.IS_DEVELOPMENT) {
    //   return;
    // }

    SmartDashboard.putData("SuperstructureVisualization", mechanism);

    var elevatorHeight = elevatorPosition;

    var elevatorPose = new Pose3d(Translation3d.kZero, Rotation3d.kZero);
    var carriagePose = new Pose3d(new Translation3d(0, 0, elevatorHeight), Rotation3d.kZero);

    var armPose =
        new Pose3d(
            new Translation3d(0, 0, elevatorHeight),
            new Rotation3d(Units.degreesToRadians(arm.getAngle()), 0, 0));
    DogLog.log(
        "SuperstructureVisualization/SuperStructure3d",
        new Pose3d[] {elevatorPose, carriagePose, armPose});

    elevator.setLength(elevatorPosition);
  }

  private MechanismVisualizer() {}
}