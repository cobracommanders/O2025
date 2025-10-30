package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public class Autos extends SubsystemBase {
    private final AutoChooser<AutoSelection> autoChooser;
    private final RequestManager requestManager;
    private final Trailblazer trailblazer;
    private final RobotCommands robotCommands;
    private boolean hasEnabledAuto = false;
    private Pair<AutoSelection, BaseAuto> selected;

    public Autos(Trailblazer trailblazer, RequestManager requestManager, RobotCommands robotCommands) {
        autoChooser = new AutoChooser<>(AutoSelection.values(), AutoSelection.FOUR_CORAL_NON_PROCESSOR);
        this.trailblazer = trailblazer;
        this.requestManager = requestManager;
        this.robotCommands = robotCommands;

        updateSelected(AutoSelection.FOUR_CORAL_NON_PROCESSOR);
    }

    public Command getAutoCommand() {
        return selected.getSecond().createAutoCommand();
    }

    public void periodic() {
        if (DriverStation.isDisabled() && !hasEnabledAuto) {
            AutoSelection currentSelection = autoChooser.getSelectedAuto();
            if (selected == null || selected.getFirst() != currentSelection) {
                updateSelected(currentSelection);
            }
        }
        if (DriverStation.isAutonomousEnabled()) {
            hasEnabledAuto = true;
        }
    }

    private void updateSelected(AutoSelection selection) {
        selected = Pair.of(selection, selection.auto.apply(requestManager, trailblazer, robotCommands));
    }
}