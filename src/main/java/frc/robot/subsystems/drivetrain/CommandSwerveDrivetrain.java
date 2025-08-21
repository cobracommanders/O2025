package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private Field2d field = new Field2d();

    // private LimelightLocalization limelightLocalization = new
    // LimelightLocalization();
    public void setYaw(Rotation2d rotation) {
        // this.getPigeon2().setYaw(angle);
        this.resetRotation(rotation);
    }

    public void setYaw(Alliance alliance) {
        setYaw((alliance == Alliance.Red) ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
    }
    // public void setYaw(Alliance alliance) {
    // if (alliance == Alliance.Red) {
    // setYaw(RedAlliancePerspectiveRotation);
    // }

    // else {
    // setYaw(BlueAlliancePerspectiveRotation);
    // }
    // }

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(
            Constants.DrivertrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(
            Constants.DrivertrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    public SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public boolean isMoving() {
        return (Math.abs(this.getState().Speeds.vxMetersPerSecond) >= 1
                || Math.abs(this.getState().Speeds.vyMetersPerSecond) >= 1
                || Math.abs(this.getState().Speeds.omegaRadiansPerSecond) >= 0.5);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        // if (Utils.isSimulation()) {
        // startSimThread();
        // }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.RobotCentric().withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.FieldCentric().withVelocityX(xLimiter.calculate(speeds.vxMetersPerSecond))
                .withVelocityY(yLimiter.calculate(speeds.vyMetersPerSecond))
                .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // SmartDashboard.putData(field);
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // AutoBuilder.configure(
        //         () -> this.getState().Pose, // Robot pose supplier
        //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         () -> this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
        //                                                               // RELATIVE ChassisSpeeds
        //         new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
        //                                         // Constants class
        //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //         ),
        //         config, // The robot configuration
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red
        //             // alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        // Reference to this subsystem to set requirements

        // if (Utils.isSimulation()) {
        // startSimThread();
        // }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    // private void startSimThread() {
    //     lastSimTime = Utils.getCurrentTimeSeconds();
    
    //     /* Run simulation at a faster rate so PID gains behave more reasonably */
    //     simNotifier =
    //         new Notifier(
    //             () -> {
    //               double currentTime = Utils.getCurrentTimeSeconds();
    //               double deltaTime = currentTime - lastSimTime;
    //               lastSimTime = currentTime;
    
    //               /* use the measured time delta, get battery voltage from WPILib */
    //               this.updateSimState(deltaTime, RobotController.getBatteryVoltage());
    //             });
    //     simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    //   }
    @Override
    public void simulationPeriodic() {
        this.updateSimState(Constants.SIM_LOOP_TIME, RobotController.getBatteryVoltage());
    }
    @Override
    public void periodic() {
        // limelightLocalization.update();
        // field.setRobotPose(this.getState().Pose);
        DogLog.log("Robot Pose", this.getState().Pose);
        
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
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
