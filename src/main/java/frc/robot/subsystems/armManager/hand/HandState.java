package frc.robot.subsystems.armManager.hand;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum HandState {
    IDLE_ALGAE(-0.12),
    IDLE_CORAL(0),
    IDLE_EMPTY(0.0),
    CLEAR_ALGAE(1.0), // Clear algae that is dropped according to sensors

    SCORE_CORAL(0.5),
    HANDOFF(-1.0),
    INTAKE_REEF_ALGAE(-.2),
    INTAKE_GROUND_ALGAE(-.75),
    SCORE_ALGAE_NET(0.8),
    SCORE_ALGAE_PROCESSOR(1.0),
    INVERTED_HANDOFF(1.0),
    LOLLIPOP(-1.0);

    private final double defaultSpeed;
    private final DoubleSubscriber tunableSpeed;

    HandState(double speed) {
        this.defaultSpeed = speed;
        this.tunableSpeed = DogLog.tunable("Hand/State/" + name(), defaultSpeed);
    }

    public double getSpeed() {
        return tunableSpeed.get();
    }
}
