package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.stateMachine.RequestManager;
import frc.robot.trailblazer.Trailblazer;

public class Autos extends SubsystemBase {
    private final AutoChooser<AutoSelection> autoChooser;
    private final RequestManager requestManager;
    private final Trailblazer trailblazer;
    private boolean hasEnabledAuto = false;
    private Pair<AutoSelection, BaseAuto> selectedRed;
    private Pair<AutoSelection, BaseAuto> selectedBlue;
    private Command autoCommand;

    public Autos(Trailblazer trailblazer, RequestManager requestManager) {
        autoChooser = new AutoChooser<>(AutoSelection.values(), AutoSelection.L4);
        this.trailblazer = trailblazer;
        this.requestManager = requestManager;

        selectedRed =
                Pair.of(
                        AutoSelection.L4,
                        AutoSelection.L4.redAuto.apply(requestManager, trailblazer));
        selectedBlue =
                Pair.of(
                        AutoSelection.L4,
                        AutoSelection.L4.blueAuto.apply(requestManager, trailblazer));
        autoCommand = createAutoCommand();
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            updateSelection();

            if (!hasEnabledAuto
                    && (RobotBase.isSimulation()
                            || DriverStation.isAutonomous()
                            || DriverStation.isFMSAttached())) {
                // Continuously reset pose
                resetPoseForAuto();
            }
        }
        if (DriverStation.isAutonomousEnabled()) {
            hasEnabledAuto = true;
        }
    }

    private void resetPoseForAuto() {
        var auto =
                FmsSubsystem.getInstance().isRedAlliance()
                        ? selectedRed.getSecond()
                        : selectedBlue.getSecond();
        var startingPose = auto.getStartingPose();
        LocalizationSubsystem.getInstance().resetPose(startingPose);
    }

    private void updateSelection() {
        // If anything about the auto selection has changed, fully recreate all the commands
        // This avoids potential errors from composed commands being used in multiple compositions
        if (selectedRed.getFirst() != autoChooser.getSelectedAuto()
                || selectedBlue.getFirst() != autoChooser.getSelectedAuto()) {
            selectedRed =
                    Pair.of(
                            autoChooser.getSelectedAuto(),
                            autoChooser
                                    .getSelectedAuto()
                                    .redAuto
                                    .apply(requestManager, trailblazer));

            selectedBlue =
                    Pair.of(
                            autoChooser.getSelectedAuto(),
                            autoChooser
                                    .getSelectedAuto()
                                    .blueAuto
                                    .apply(requestManager, trailblazer));

            autoCommand = createAutoCommand();
        }
    }

    private Command createAutoCommand() {
        return Commands.either(
                        selectedRed.getSecond().getAutoCommand(),
                        selectedBlue.getSecond().getAutoCommand(),
                        FmsSubsystem.getInstance()::isRedAlliance)
                .withName(
                        selectedRed.getSecond().name() + "_or_" + selectedBlue.getSecond().name());
    }
}
