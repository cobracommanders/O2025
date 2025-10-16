package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.trailblazer.Trailblazer;
import frc.robot.trailblazer.TrailblazerPathLogger;

public abstract class BaseAuto implements NamedAuto {
    protected final RequestManager requestManager;
    protected final Trailblazer trailblazer;
    //  protected final RobotCommands actions;
    protected final AutoBlocks blocks;
    protected final AutoTiming timing;
    private final String autoName;
    private final Command autoCommand;

    protected BaseAuto(RequestManager robotManager, Trailblazer trailblazer) {
        this.requestManager = robotManager;
        this.trailblazer = trailblazer;
        //    actions = RobotCommands.getInstance();
        blocks = new AutoBlocks(requestManager, trailblazer);

        var className = this.getClass().getSimpleName();
        autoName = className.substring(className.lastIndexOf('.') + 1);
        timing = new AutoTiming(autoName);

        autoCommand = createFullAutoCommand();
    }

    protected abstract Pose2d getStartingPose();

    protected abstract Command createAutoCommand();

    /** Returns the name of this auto. */
    @Override
    public String name() {
        return autoName;
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    private Command createFullAutoCommand() {
        TrailblazerPathLogger.markAuto(this);
        // We continuously reset the pose anyway, but doing it here should be fine
        // It's basically free as long as we aren't updating the IMU
        return timing.time(
                        "TotalTime",
                        // TODO: Seems like this doesn't run or runs incorrectly in sim
                        Commands.runOnce(
                                () ->
                                        LocalizationSubsystem.getInstance()
                                                .resetPose(getStartingPose())),
                        createAutoCommand())
                .finallyDo(
                        interrupted -> {
                            // Stop driving once the auto finishes
                            DriveSubsystem.getInstance()
                                    .setFieldRelativeAutoSpeeds(new ChassisSpeeds());

                            // Check if we are enabled, since auto commands are cancelled during
                            // disable
                            if (interrupted && DriverStation.isAutonomousEnabled()) {
                                DogLog.logFault("Auto command interrupted outside teleop");
                            }
                        })
                .withName(autoName + "Command");
    }
}
