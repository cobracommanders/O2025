package frc.robot.fms;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class FmsSubsystem extends SubsystemBase {
    private boolean disabled= true;
    private boolean teleopEnabled = false;
    private boolean autoEnabled = false;
    private Alliance alliance = Alliance.Red;

    public FmsSubsystem() {
    }

    public boolean isDisabled() {
        return disabled;
    }

    public boolean isTeleop() {
        return teleopEnabled;
    }

    public boolean isAutonomous() {
        return autoEnabled;
    }

    public boolean isRedAlliance() {
        return alliance == Alliance.Red;
    }

    @Override
    public void periodic() {
        disabled = DriverStation.isDisabled();
        teleopEnabled = DriverStation.isTeleop();
        autoEnabled = DriverStation.isAutonomous();
        alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
    }

    private static FmsSubsystem instance;

    public static FmsSubsystem getInstance() {
        if (instance == null)
            instance = new FmsSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}