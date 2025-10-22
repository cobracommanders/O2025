package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveToPoseRequest implements SwerveRequest {
    private final FieldCentric fieldCentric = new FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final ProfiledPIDController linearController = new ProfiledPIDController(
            3.7,
            0,
            0.05,
            new TrapezoidProfile.Constraints(
                    5.0, 4.0
            )
    );

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            4.0,
            0,
            0.2,
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(360.0), Units.degreesToRadians(360.0)
            )
    );

    public DriveToPoseRequest() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(0.01);
    }

    public Pose2d Target = Pose2d.kZero;

    public DriveToPoseRequest withTarget(Pose2d target) {
        this.Target = target;
        return this;
    }

    private static final double ROTATION_FEED_FORWARD = 0.01;

    @Override
    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        double rotationSpeed = thetaController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                Target.getRotation().getRadians()
        );

        if (!MathUtil.isNear(Target.getRotation().getDegrees(), parameters.currentPose.getRotation().getDegrees(), 1.0)) {
            rotationSpeed += Math.copySign(Units.rotationsToRadians(ROTATION_FEED_FORWARD), rotationSpeed);
        }

        double xError = Target.getX() - parameters.currentPose.getX();
        double yError = Target.getY() - parameters.currentPose.getY();
        double distance = Math.hypot(xError, yError);

        double m_cos;
        double m_sin;

        double magnitude = Math.hypot(xError, yError);
        if (magnitude > 1.0E-6) {
            m_cos = xError / magnitude;
            m_sin = yError / magnitude;
        } else {
            m_cos = 1.0F;
            m_sin = 0.0F;
            MathSharedStore.reportError("x and y components of Rotation2d are zero\n", Thread.currentThread().getStackTrace());
        }

        // 0.02 is added as a slight feedforward to make sure the robot always has the minimum power needed to move
        // Completely arbitrary and may need tuning or removal at some point

        double velocity = -linearController.calculate(distance, 0.0) + 0.02;

        double vx = velocity * m_cos;
        double vy = velocity * m_sin;

        return fieldCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(rotationSpeed).apply(parameters, modulesToApply);
    }
}
