package frc.robot.subsystems.armManager.arm;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
    START_POSITION(0.25),
    IDLE_ALGAE(0.25),
    IDLE_CORAL_UP(0.25),
    INTAKE_GROUND_ALGAE(-0.032),
    HANDOFF_RIGHT(-0.235),
    HANDOFF_LEFT(-0.265),
    HANDOFF_MIDDLE(-0.26),
    CLIMB(0.0),
    LOLLIPOP(-0.056),
    ALGAE_PROCESSOR(0),

    RIGHT(0.0),
    LEFT(invertPosition(RIGHT)),
    UP(0.25),
    DOWN(-0.25),

    INTAKE_HIGH_REEF_ALGAE_RIGHT(0 + Units.degreesToRotations(1)),
    INTAKE_HIGH_REEF_ALGAE_LEFT(invertPosition(INTAKE_HIGH_REEF_ALGAE_RIGHT)),
    INTAKE_LOW_REEF_ALGAE_RIGHT(0 + Units.degreesToRotations(1)),
    INTAKE_LOW_REEF_ALGAE_LEFT(invertPosition(INTAKE_LOW_REEF_ALGAE_RIGHT)),
    ALGAE_NET_RIGHT(0.17),
    ALGAE_NET_LEFT(invertPosition(ALGAE_NET_RIGHT)),
    PREPARE_L4_RIGHT(0.06),
    PREPARE_L4_LEFT(invertPosition(PREPARE_L4_RIGHT)),
    PREPARE_L3_RIGHT(0.08),
    PREPARE_L3_LEFT(invertPosition(PREPARE_L3_RIGHT)),
    PREPARE_L2_RIGHT(0.08),
    PREPARE_L2_LEFT(invertPosition(PREPARE_L2_RIGHT)),
    SCORE_L4_RIGHT(0.0),
    SCORE_L4_LEFT(invertPosition(SCORE_L4_RIGHT)),
    SCORE_L3_RIGHT(0.01),
    SCORE_L3_LEFT(invertPosition(SCORE_L3_RIGHT)),
    SCORE_L2_RIGHT(0.01),
    SCORE_L2_LEFT(invertPosition(SCORE_L2_RIGHT)),
    CUSTOM(0.0); // Set in the arm class

    private final double defaultPosition;
    private final DoubleSubscriber tunablePosition;

    ArmState(double position) {
        this.defaultPosition = position;
        this.tunablePosition = DogLog.tunable("Arm/State/" + name(), defaultPosition);
    }

    public double getPosition() {
        return tunablePosition.get();
    }

    private static double invertPosition(ArmState position) {
        return invertPosition(position.defaultPosition);
    }

    public static double invertPosition(double position) {
        return -position + 0.5;
    }
}
