package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
        DogLog.log(
        "SuperstructureVisualization/Superstructure3d",
        new Pose3d[] {carriagePose, armPose, groundPivotPose});
    }

    public static void publishCoral(Pose3d pose) {

    }
    
    public static void publishAlgae(Pose3d pose) {

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
