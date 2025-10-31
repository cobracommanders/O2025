package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public abstract class BaseAuto implements NamedAuto {
    protected final RequestManager requestManager;
    protected final Trailblazer trailblazer;
    protected final AutoBlocks blocks;
    private final String autoName;
    private final Command autoCommand;
    protected final RobotCommands robotCommands;

    protected BaseAuto(RequestManager robotManager, Trailblazer trailblazer, RobotCommands robotCommands) {
        this.requestManager = robotManager;
        this.trailblazer = trailblazer;
        this.robotCommands = robotCommands;
        blocks = new AutoBlocks(requestManager, trailblazer);

        var className = this.getClass().getSimpleName();
        autoName = className.substring(className.lastIndexOf('.') + 1);

        autoCommand = createAutoCommand();
    }

    protected abstract Command createAutoCommand();

    /**
     * Returns the name of this auto.
     */
    @Override
    public String name() {
        return autoName;
    }

    public Command getAutoCommand() {
        return autoCommand;
    }
}