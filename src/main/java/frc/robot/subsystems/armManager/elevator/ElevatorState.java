package frc.robot.subsystems.armManager.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
    IDLE(0.1),
    IDLE_CORAL_UP(0.4),
    HIGH_REEF_ALGAE(0.95),
    LOW_REEF_ALGAE(0.53),
    GROUND_ALGAE(0),
    ALGAE_NET(1.38),
    ALGAE_PROCESSOR(0.1),
    HANDOFF(0.775 - Units.inchesToMeters(0.75)), // Actual handoff position
    IDLE_EMPTY(0.825), // State above the intake that allows the intake to move in/out freely
    LOLLIPOP(0),
    PREPARE_L4(1.3),
    PREPARE_L3(0.765),
    PREPARE_L2(0.3),
    SCORE_L4(1.13),
    SCORE_L3(0.624),
    SCORE_L2(0.3),
    CUSTOM(0.0); // Set in the elevator class


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
