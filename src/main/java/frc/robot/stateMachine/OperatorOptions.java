package frc.robot.stateMachine;

public class OperatorOptions {

    public ScoreLocation scoreLocation;
    public AlgaeIntakeLevel algaeIntakeLevel;

    public OperatorOptions() {
        this.scoreLocation = ScoreLocation.L3;
        this.algaeIntakeLevel = AlgaeIntakeLevel.GROUND;
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
        GROUND,
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
