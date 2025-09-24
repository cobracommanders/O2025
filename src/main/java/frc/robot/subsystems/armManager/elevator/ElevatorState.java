package frc.robot.subsystems.armManager.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
    IDLE(0.1),
    HIGH_REEF_ALGAE(0.9),
    LOW_REEF_ALGAE(0.53),
    GROUND_ALGAE(0),
    ALGAE_NET(1.38),
    ALGAE_PROCESSOR(0.1),
    HANDOFF(0.75),
    HANDOFF_CORAL_MODE(0.85),
    LOLLIPOP(0),
    PREPARE_L4(1.3),
    PREPARE_L3(0.765),
    PREPARE_L2(0.41),
    SCORE_L4(1.15),
    SCORE_L3(0.624),
    SCORE_L2(0.3);


    private final double defaultPosition;
    private final DoubleSubscriber tunablePosition;

    ElevatorState(double position) {
        this.defaultPosition = position;
        this.tunablePosition = DogLog.tunable("Elevator/State/" + name(), defaultPosition);
    }

    public double getPosition() {
        return tunablePosition.get();
    }
}
