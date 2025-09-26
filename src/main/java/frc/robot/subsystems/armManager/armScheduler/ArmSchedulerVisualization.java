package frc.robot.subsystems.armManager.armScheduler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmSchedulerVisualization {
    private final double windowSize = 2.0;
    private final double windowCenter = windowSize / 2.0;
    private final Mechanism2d mechanism2d;
    private final MechanismRoot2d armRoot;
    private final MechanismRoot2d driveRoot;
    private final MechanismRoot2d intakeRoot;
    private final MechanismLigament2d drivetrain;
    private final MechanismLigament2d armLine1;
    private final MechanismLigament2d armLine2;
    private final MechanismLigament2d intake;

    public ArmSchedulerVisualization(double driveWidth, double driveHeight, double intakeWidth, double finalIntakeHeight) {
        this.mechanism2d = new Mechanism2d(windowSize, windowSize);
        this.armRoot = mechanism2d.getRoot("Arm", 0.0, 0.0);
        this.driveRoot = mechanism2d.getRoot("Drive", windowCenter - driveWidth / 2.0, driveHeight);
        this.intakeRoot = mechanism2d.getRoot("Intake", windowCenter - intakeWidth / 2.0, 0.0);
        this.drivetrain = new MechanismLigament2d("Drivetrain", driveWidth, 0.0, 2.0, new Color8Bit(Color.kRed));
        this.armLine1 = new MechanismLigament2d("Arm1", 0.0, 0.0, 1, new Color8Bit(Color.kWhite));
        this.armLine2 = new MechanismLigament2d("Arm2", 0.0, 0.0, 1, new Color8Bit(Color.kWhite));
        this.intake = new MechanismLigament2d("Intake", intakeWidth, 0.0, 1, new Color8Bit(Color.kRed));

        driveRoot.append(drivetrain);
        armRoot.append(armLine1);
        armRoot.append(armLine2);
        intakeRoot.append(intake);
    }

    public Mechanism2d getMechanism2d() {
        return mechanism2d;
    }

    public void drawArm(double pivotY, double armX1, double armY1, double armX2, double armY2) {
        armRoot.setPosition(windowCenter, pivotY);
        armLine1.setAngle(Units.radiansToDegrees(Math.atan2(armY1 - pivotY, armX1)));
        armLine1.setLength(Math.hypot(armX1, armY1 - pivotY));
        armLine2.setAngle(Units.radiansToDegrees(Math.atan2(armY2 - pivotY, armX2)));
        armLine2.setLength(Math.hypot(armX2, armY2 - pivotY));
    }

    public void drawIntake(double intakeWidth, double finalIntakeHeight) {
        intakeRoot.setPosition(windowCenter - intakeWidth / 2.0, finalIntakeHeight);
        intake.setColor(new Color8Bit(Color.kRed));
    }
}
