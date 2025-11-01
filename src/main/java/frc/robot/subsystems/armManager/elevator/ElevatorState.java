package frc.robot.subsystems.armManager.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
    IDLE(0.1),
    CLIMB(0.0),
    IDLE_CORAL_UP(0.4),
    HIGH_REEF_ALGAE(0.9),
    LOW_REEF_ALGAE(0.53),
    GROUND_ALGAE(0.16),
    ALGAE_NET(1.38 + Units.inchesToMeters(1.5)),
    ALGAE_PROCESSOR(0.1),
    HANDOFF(0.785 - Units.inchesToMeters(0.75)), // Actual handoff position
    IDLE_EMPTY(0.825), // State above the intake that allows the intake to move in/out freely
    LOLLIPOP(0),
    PREPARE_L4(1.418),
    PREPARE_L3(0.765),
    PREPARE_L2(0.35),
    SCORE_L4(1.25 + Units.inchesToMeters(1.5)),
    SCORE_L3(0.634),
    SCORE_L2(0.219),
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
