package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.fasterxml.jackson.databind.node.NullNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.fms.FmsSubsystem;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public void setYaw(Rotation2d rotation) {
        this.resetRotation(rotation);
    }

    public void setYaw(Alliance alliance) {
        setYaw((alliance == Alliance.Red) ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
    }
    public void setYawFromFMS() {
        setYaw((FmsSubsystem.getInstance().isRedAlliance()) ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
    }

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(
            Constants.DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(
            Constants.DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    public SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        
    }

    public boolean isMoving() {
        return (Math.abs(this.getState().Speeds.vxMetersPerSecond) >= 1
                || Math.abs(this.getState().Speeds.vyMetersPerSecond) >= 1
                || Math.abs(this.getState().Speeds.omegaRadiansPerSecond) >= 0.5);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
    @Override
    public void simulationPeriodic() {
        this.updateSimState(Constants.SIM_LOOP_TIME, RobotController.getBatteryVoltage());
    }
    @Override
    public void update() {
        DogLog.log("Robot Pose", this.getState().Pose);
        
        if (!hasAppliedOperatorPerspective || FmsSubsystem.getInstance().isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    private static CommandSwerveDrivetrain instance;

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null)
            instance = TunerConstants.createDrivetrain();
        return instance;
    }
}
