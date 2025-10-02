package frc.robot.stateMachine;

import dev.doglog.DogLog;
import frc.robot.FieldConstants;

public class OperatorOptions {

    public ScoreLocation scoreLocation;
    public AlgaeIntakeLevel algaeIntakeLevel;

    private OperatorOptions() {
        this.scoreLocation = ScoreLocation.L4;
        this.algaeIntakeLevel = AlgaeIntakeLevel.GROUND_ALGAE;
        DogLog.log("Robot/ScoreLocation", "L4");
        DogLog.log("Robot/CoralMode", "NORMAL");
        DogLog.log("Robot/AlgaeLocation", "GROUND");
    }

    public enum ScoreLocation {
        L1,
        L2,
        L3,
        L4,
        BARGE,
        PROCESSOR;
        FieldConstants.PipeScoringLevel toPipeScoringLevelOrL4() {
            return switch (this) {
                case L2 -> FieldConstants.PipeScoringLevel.L2;
                case L3 -> FieldConstants.PipeScoringLevel.L3;
                case L4 -> FieldConstants.PipeScoringLevel.L4;
                case L1, BARGE, PROCESSOR -> FieldConstants.PipeScoringLevel.L4;
            };
        }
    }

    public enum AlgaeIntakeLevel {
        GROUND_ALGAE,
        LOW_REEF,
        HIGH_REEF
    }

    private static OperatorOptions instance;

    public static OperatorOptions getInstance() {
        if (instance == null)
            instance = new OperatorOptions(); // Make sure there is an instance (this will only run once)
        return instance;
    }


}
