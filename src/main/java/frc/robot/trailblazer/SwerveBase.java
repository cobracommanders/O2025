package frc.robot.trailblazer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.AutoCommands;

public interface SwerveBase extends Subsystem {
  void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds);

  ChassisSpeeds getFieldRelativeSpeeds();
}