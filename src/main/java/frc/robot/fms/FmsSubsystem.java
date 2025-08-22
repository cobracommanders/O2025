package frc.robot.fms;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class FmsSubsystem extends SubsystemBase {

    public FmsSubsystem() {
    }

    @Override
    public void periodic() {
        DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
    }

    private static FmsSubsystem instance;

    public static FmsSubsystem getInstance() {
        if (instance == null)
            instance = new FmsSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}