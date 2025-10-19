package frc.robot.stateMachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.autoAlign.ReefPipe;
import frc.robot.autoAlign.ReefPipeLevel;

public class OperatorOptions {
    public CoralScoreLocation coralScoreLocation = CoralScoreLocation.L4;
    public AlgaeScoreLocation algaeScoreLocation = AlgaeScoreLocation.BARGE;
    public AlgaeIntakeLevel algaeIntakeLevel = AlgaeIntakeLevel.LOW_REEF;

    private OperatorOptions() {}

    public enum AlgaeScoreLocation {
        BARGE, PROCESSOR;
    }

    public enum CoralScoreLocation {
        L1,
        L2,
        L3,
        L4;

        FieldConstants.PipeScoringLevel toPipeScoringLevelOrL4() {
            return switch (this) {
                case L2 -> FieldConstants.PipeScoringLevel.L2;
                case L3 -> FieldConstants.PipeScoringLevel.L3;
                case L4 -> FieldConstants.PipeScoringLevel.L4;
                default -> FieldConstants.PipeScoringLevel.L4;
            };
        }
        ReefPipeLevel toReefPipeLevelOrL4() {
            return switch (this) {
                case L1 -> ReefPipeLevel.L1;
                case L2 -> ReefPipeLevel.L2;
                case L3 -> ReefPipeLevel.L3;
                case L4 -> ReefPipeLevel.L4;
                default -> ReefPipeLevel.L4;
            };
        }
    }

    public enum AlgaeIntakeLevel {
        GROUND_ALGAE,
        LOW_REEF,
        HIGH_REEF
        }

    public Command setProcessorCommand() {
        return Commands.runOnce(() -> algaeScoreLocation = AlgaeScoreLocation.PROCESSOR).withName("setProcessor");
    }

    public Command setBargeCommand() {
        return Commands.runOnce(() -> algaeScoreLocation = AlgaeScoreLocation.BARGE).withName("setBarge");
    }

    public Command setL1Command() {
        return Commands.runOnce(() -> coralScoreLocation = CoralScoreLocation.L1).withName("setL1");
    }

    public Command setL2Command() {
        return Commands.runOnce(() -> coralScoreLocation = CoralScoreLocation.L2).withName("setL2");
    }

    public Command setL3Command() {
        return Commands.runOnce(() -> coralScoreLocation = CoralScoreLocation.L3).withName("setL3");
    }

    public Command setL4Command() {
        return Commands.runOnce(() -> coralScoreLocation = CoralScoreLocation.L4).withName("setL4");
    }

    public Command setHighReefAlgaeCommand() {
        return Commands.runOnce(() -> algaeIntakeLevel = AlgaeIntakeLevel.HIGH_REEF).withName("setHighReefAlgae");
    }

    public Command setLowReefAlgaeCommand() {
        return Commands.runOnce(() -> algaeIntakeLevel = AlgaeIntakeLevel.LOW_REEF).withName("setLowReefAlgae");
    }

    public Command setGroundAlgaeCommand() {
        return Commands.runOnce(() -> algaeIntakeLevel = AlgaeIntakeLevel.GROUND_ALGAE).withName("setGroundAlgae");
    }

    public boolean isCoralScoringL1() {
        return coralScoreLocation == CoralScoreLocation.L1;
    }

    public FieldConstants.PipeScoringLevel getPipeScoringLevelOrL4() {
        return coralScoreLocation.toPipeScoringLevelOrL4();
    }

    public ReefPipeLevel getReefPipeLevelOrL4() {
        return coralScoreLocation.toReefPipeLevelOrL4();
    }

    public AlgaeIntakeLevel getAlgaeIntakeLevel(){
        return algaeIntakeLevel;
    }

    private static OperatorOptions instance;

    public static OperatorOptions getInstance() {
        if (instance == null)
            instance = new OperatorOptions(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
