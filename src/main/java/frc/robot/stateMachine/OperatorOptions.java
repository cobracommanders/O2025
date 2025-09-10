package frc.robot.stateMachine;

import dev.doglog.DogLog;

public class OperatorOptions {

    public ScoreLocation scoreLocation;
    public AlgaeIntakeLevel algaeIntakeLevel;
    public CoralMode coralMode;

    public OperatorOptions() {
        this.scoreLocation = ScoreLocation.L4;
        this.algaeIntakeLevel = AlgaeIntakeLevel.GROUND_ALGAE;
        DogLog.log("Robot/ScoreLocation", "L4");
        this.coralMode = CoralMode.CORAL_MODE;
        DogLog.log("Robot/ScoreLocation", "L3");
        DogLog.log("Robot/AlgaeLocation", "GROUND");

    }

    public enum ScoreLocation {
        L1,
        L2,
        L3,
        L4,
        BARGE,
        PROCESSOR
    }
    public enum AlgaeIntakeLevel {
        GROUND_ALGAE,
        LOW_REEF,
        HIGH_REEF
    }
    public enum CoralMode {
        CORAL_MODE,
        NORMAL_MODE
    }

    private static OperatorOptions instance;

    public static OperatorOptions getInstance() {
        if (instance == null)
            instance = new OperatorOptions(); // Make sure there is an instance (this will only run once)
        return instance;
    }


}
