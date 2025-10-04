package frc.robot.mechanism_visualizer;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetector;

public class MechanismVisualizer {
    static Mechanism2d armManager = new Mechanism2d(2, 3);
    static MechanismRoot2d armRoot = armManager.getRoot("ArmManager", 1, 0.5);
    static MechanismLigament2d elevator = armRoot.append(new MechanismLigament2d("Elevator", 1.5, 90));
    static MechanismLigament2d arm = elevator.append(new MechanismLigament2d("Arm", 0.7, 90, 6, new Color8Bit(Color.kPurple)));

    static Mechanism2d groundManager = new Mechanism2d(1, 1);
    static MechanismRoot2d groundRoot = groundManager.getRoot("GroundManager", 0.3, 0.2);
    static MechanismLigament2d groundPivot = groundRoot.append(new MechanismLigament2d("GroundPivot", 0.2, 90, 6, new Color8Bit(Color.kGreen)));

    public static void publishData() {
        // publish Mechanism2d
        SmartDashboard.putData("ArmManagerVisualizer", armManager);
        SmartDashboard.putData("GroundManagerVisualizer", groundManager);
        // publish 3d custom components
        var elevatorHeight = elevator.getLength();
        var armAngle = arm.getAngle() - 90;
        var groundPivotAngle = 180 + groundPivot.getAngle();

        var minCarriageHeight = 0.304800;

        var carriageOffset = -0.171450;

        var carriagePose = new Pose3d(new Translation3d(carriageOffset, 0, minCarriageHeight + elevatorHeight), Rotation3d.kZero);
        var armPose = new Pose3d(
                new Translation3d(carriageOffset, 0, minCarriageHeight + elevatorHeight),
                new Rotation3d(Units.degreesToRadians(armAngle), 0, 0)
        );
        var groundPivotPose = new Pose3d(
                new Translation3d(0.292100, 0, 0.1778),
                new Rotation3d(0, Units.degreesToRadians(groundPivotAngle), 0)
        );

        if (Robot.isSimulation()) {
            Pose3d robotPose3d = new Pose3d(LocalizationSubsystem.getInstance().getPose());

            if (CoralDetector.getInstance().hasCoral()) {
                Transform3d intakeCoralTransform = switch (CoralDetector.getInstance().getState()) {
                    case LEFT -> new Transform3d(0.25, 0.1, 0.0, new Rotation3d(0.0, 0.0, Math.PI / 2));
                    case RIGHT -> new Transform3d(0.25, -0.1, 0.0, new Rotation3d(0.0, 0.0, Math.PI / 2));
                    default -> new Transform3d(0.25, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI / 2));
                };

                DogLog.log("MechanismVisualizer/IntakeCoral",
                        new Pose3d[]{
                                robotPose3d
                                        .transformBy(new Transform3d(Pose3d.kZero, groundPivotPose.transformBy(intakeCoralTransform)))}
                );
            } else {
                DogLog.log("MechanismVisualizer/IntakeCoral", new Pose3d[]{});
            }

            // Display arm coral
            Transform3d armCoralTransform = new Transform3d(0.35, -0.55, 0.0, new Rotation3d(0.0, Math.PI / 2, 0.0));
            Transform3d armAlgaeTransform = new Transform3d(0.35, -0.7, 0.0, new Rotation3d(0.0, 0.0, 0.0));
            switch (Robot.armManager.getSimHandGamePiece()) {
                case CORAL -> {
                    DogLog.log("MechanismVisualizer/ArmCoral", new Pose3d[]{robotPose3d.transformBy(new Transform3d(Pose3d.kZero, armPose.transformBy(armCoralTransform)))});
                    DogLog.log("MechanismVisualizer/ArmAlgae", new Pose3d[]{});
                }
                case ALGAE -> {
                    DogLog.log("MechanismVisualizer/ArmAlgae", new Pose3d[]{robotPose3d.transformBy(new Transform3d(Pose3d.kZero, armPose.transformBy(armAlgaeTransform)))});
                    DogLog.log("MechanismVisualizer/ArmCoral", new Pose3d[]{});
                }
                case NONE -> {
                    DogLog.log("MechanismVisualizer/ArmCoral", new Pose3d[]{});
                    DogLog.log("MechanismVisualizer/ArmAlgae", new Pose3d[]{});
                }
            }
        }

        DogLog.log(
                "MechanismVisualizer/Superstructure3d",
                new Pose3d[]{carriagePose, armPose, groundPivotPose});
    }

    public static void setElevatorPosition(double length) {
        elevator.setLength(0.05 + length);
    }

    public static void setArmPosition(double rotations) {
        arm.setAngle(elevator.getAngle() - rotations * 360);
    }

    public static void setGroundPivotPosition(double rotations) {
        groundPivot.setAngle(180 - rotations * 360);
    }
}
