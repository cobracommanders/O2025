package frc.robot.stateMachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.autoAlign.AutoAlign;
import frc.robot.autoAlign.ReefPipeLevel;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.subsystems.armManager.ArmManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.ground_manager.GroundManager;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class RequestManager {
    private final Climber climber;
    private final ArmManager.CommandWrapper armCommands;
    private final GroundManager.CommandWrapper groundCommands;

    public RequestManager(ArmManager armManager, GroundManager groundManager, Climber climber) {
        this.climber = climber;
        this.armCommands = new ArmManager.CommandWrapper(armManager);
        this.groundCommands = new GroundManager.CommandWrapper(groundManager);
    }

    public RobotScoringSide reefRobotSide() {
        return AutoAlign.getScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
    }

    public RobotScoringSide netRobotSide() {
        return AutoAlign.getNetScoringSideFromRobotPose(LocalizationSubsystem.getInstance().getPose());
    }

    public boolean algaeIntakeHeightIsTop() {
        if (FeatureFlags.AUTO_ALGAE_INTAKE_HEIGHT.getAsBoolean()) {
            return AutoAlign.getInstance().getClosestReefSide().algaeHeight == ReefPipeLevel.L3;
        } else {
            return OperatorOptions.getInstance().algaeIntakeLevel == OperatorOptions.AlgaeIntakeLevel.HIGH_REEF;
        }
    }

    public Command resetArmGamePieceAndIdle() {
        return armCommands.idleClearGamePieceAndAwaitReady();
    }

    public Command idleArm() {
        return armCommands.idleAndAwaitReady();
    }

    public Command idleGround() {
        return groundCommands.idleAndAwaitReady();
    }

    public Command idleAll() {
        return Commands.parallel(idleArm(), idleGround());
    }

    public Command algaeNetScore(Supplier<RobotScoringSide> side, BooleanSupplier confirmation) {
        return armCommands.requestAlgaeNetPrepareAndAwaitReady(side)
                .andThen(armCommands.doNothing().until(confirmation))
                .andThen(armCommands.executeAlgaeNetScoreAndAwaitIdle());
    }

    public Command algaeProcessorScore(BooleanSupplier confirmation) {
        return armCommands.requestAlgaeProcessorPrepareAndAwaitReady()
                .andThen(armCommands.doNothing().until(confirmation))
                .andThen(armCommands.executeAlgaeProcessorScoreAndAwaitIdle());
    }

    public Command prepareCoralScoreAndAwaitReady(
            Supplier<RobotScoringSide> scoringSide,
            Supplier<FieldConstants.PipeScoringLevel> scoringLevel
    ) {
        return armCommands.requestCoralPrepareAndAwaitReady(scoringSide, scoringLevel);
    }

    public Command prepareCoralScoreAndAwaitReady() {
        return prepareCoralScoreAndAwaitReady(
                this::reefRobotSide,
                () -> OperatorOptions.getInstance().scoreLocation.toPipeScoringLevelOrL4()
        );
    }

    public Command prepareCoralScoreAndAwaitReady(FieldConstants.PipeScoringLevel scoringLevel) {
        return prepareCoralScoreAndAwaitReady(
                this::reefRobotSide,
                () -> scoringLevel
        );
    }

    public Command executeCoralScoreAndAwaitIdleOrAuto() {
        return armCommands.executeCoralScoreAndAwaitIdleOrAuto();
    }

    /**
     * Request ground algae intake and await game piece.
     */
    public Command groundAlgaeIntake() {
        return armCommands.requestGroundAlgaeIntakeAndAwaitGamePiece();
    }

    public Command highReefAlgaeIntake(Supplier<RobotScoringSide> side) {
        return armCommands.requestAlgaeReefIntakeAndAwaitIdle(side, true);
    }

    public Command lowReefAlgaeIntake(Supplier<RobotScoringSide> side) {
        return armCommands.requestAlgaeReefIntakeAndAwaitIdle(side, false);
    }

    public Command reefAlgaeIntake() {
        return Commands.either(
                highReefAlgaeIntake(this::reefRobotSide),
                lowReefAlgaeIntake(this::reefRobotSide),
                this::algaeIntakeHeightIsTop
        );
    }

    public Command prepareLollipopAndAwaitReady() {
        return armCommands
                .requestLollipopIntakeAndAwaitReady()
                .onlyIf(DriverStation::isAutonomous);
    }

    public Command scoreL1(BooleanSupplier confirmation) {
        return groundCommands.prepareL1AndAwaitReady()
                .andThen(waitUntil(confirmation))
                .andThen(groundCommands.executeL1ScoreAndAwaitIdle());
    }

    public Command coralIntakeUntilPiece() {
        return groundCommands.intakeUntilPiece()
                .andThen(handoffRequest())
                .withName("coralIntakeRequest");
    }

    public Command climbRequest() {
        return Commands.parallel(groundCommands.climbAndDoNothing(), armCommands.requestClimbAndDoNothing())
                .andThen(climber.runOnce(() -> climber.setState(ClimberStates.DEPLOYING)));
    }

    public Command handoffRequest() {
        return groundCommands.requestHandoffAndAwaitReady()
                .andThen(armCommands.requestHandoffAndAwaitReady())
                // Request execution once both mechanisms are positioned
                .andThen(Commands.parallel(
                        groundCommands.executeHandoff(),
                        armCommands.executeHandoff()))
                // Wait for handoff time
                .andThen(waitSeconds(0.5)) // TODO Potentially incorporate canranges for extra speed
                .andThen(idleAll())
                .onlyIf(armCommands::currentGamePieceIsNone)
                .withName("handoffRequest");
    }

    public Command invertedHandoffRequest() {
        return groundCommands.requestInvertedHandoffAndAwaitReady()
                .andThen(armCommands.requestInvertedHandoffAndAwaitReady())
                // Request execution once both mechanisms are positioned
                .andThen(Commands.parallel(
                        groundCommands.executeInvertedHandoff(),
                        armCommands.executeInvertedHandoff()))
                // Wait for handoff time
                .andThen(waitSeconds(0.5).withDeadline(groundCommands.awaitGamePieceFromIntaking()))
                .andThen(idleAll())
                .onlyIf(armCommands::currentGamePieceIsNone)
                .withName("invertedHandoffRequest");
    }
}
