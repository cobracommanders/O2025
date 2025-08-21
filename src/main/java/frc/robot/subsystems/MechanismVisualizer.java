package frc.robot.subsystems;

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

    static {
        SmartDashboard.putData("ArmManagerVisualizer", armManager);
        SmartDashboard.putData("GroundManagerVisualizer", groundManager);
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
