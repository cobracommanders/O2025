package frc.robot.subsystems.armManager;

import frc.robot.FieldConstants;
import frc.robot.autoAlign.RobotScoringSide;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorState;

import static frc.robot.subsystems.armManager.ArmManagerState.HandGamePieceState.*;

public enum ArmManagerState {
    // Start position
    START_POSITION(CORAL), // Start position of the robot, holding a coral in the claw

    /*
     * IDLE STATES
     *
     * PREPARE_IDLE_XXX -> IDLE_XXX when At Position
     * IDLE_ALGAE -> IDLE_ALGAE_DROPPED when No Algae in Hand
     * IDLE_ALGAE_DROPPED -> PREPARE_IDLE_EMPTY when Slight Delay
     */
    PREPARE_IDLE_EMPTY(NONE), // Transition state
    PREPARE_IDLE_ALGAE(ALGAE), // Transition state
    PREPARE_IDLE_CORAL(CORAL), // Transition state

    IDLE_EMPTY(NONE), // Waiting for a coral in the ground intake
    IDLE_ALGAE(ALGAE), // Idle holding algae
    IDLE_CORAL(CORAL), // Idle holding coral

    IDLE_ALGAE_DROPPED(NONE), // Algae missing according to sensors, spins wheels to make sure it's actually gone and not going to collide with anything


    /*
     * HANDOFF STATES
     *
     * IDLE_EMPTY -> PREEMPTIVE_HANDOFF_<side> when Coral Detected in Intake
     * PREEMPTIVE_HANDOFF_<side> -> PREPARE_IDLE_EMPTY when No Coral Detected in Intake
     *
     * PREEMPTIVE_HANDOFF_<side> -> PREEMPTIVE_HANDOFF<side> when Coral Position Changes
     *
     * PREEMPTIVE_HANDOFF_<side> -> PREPARE_HANDOFF_<side> when External Signal
     *
     * PREPARE_HANDOFF_<side> -> READY_HANDOFF_<side> when At Position
     * READY_HANDOFF_<side> -> EXECUTE_HANDOFF_<side> when External Signal
     * EXECUTE_HANDOFF_<side> -> PREPARE_IDLE_CORAL when External Signal
     */
    PREEMPTIVE_HANDOFF_LEFT(NONE), // Prepare above intake
    PREEMPTIVE_HANDOFF_MIDDLE(NONE), // Prepare above intake
    PREEMPTIVE_HANDOFF_RIGHT(NONE), // Prepare above intake
    PREPARE_HANDOFF_LEFT(NONE), // Transition state
    PREPARE_HANDOFF_MIDDLE(NONE), // Transition state
    PREPARE_HANDOFF_RIGHT(NONE), // Transition state
    READY_HANDOFF_LEFT(NONE), // Wait for an external signal
    READY_HANDOFF_MIDDLE(NONE), // Wait for an external signal
    READY_HANDOFF_RIGHT(NONE), // Wait for an external signal
    EXECUTE_HANDOFF_LEFT(CORAL), // Execute handoff
    EXECUTE_HANDOFF_MIDDLE(CORAL), // Execute handoff
    EXECUTE_HANDOFF_RIGHT(CORAL), // Execute handoff


    /*
     * INVERTED HANDOFF STATES
     *
     * PREPARE_INVERTED_HANDOFF -> READY_INVERTED_HANDOFF when At Position
     * READY_INVERTED_HANDOFF -> EXECUTE_INVERTED_HANDOFF when External Signal
     * EXECUTE_INVERTED_HANDOFF -> IDLE_EMPTY when External Signal
     */
    PREPARE_INVERTED_HANDOFF(CORAL),
    READY_INVERTED_HANDOFF(CORAL),
    EXECUTE_INVERTED_HANDOFF(NONE),


    /*
     * CORAL SCORE STATES
     *
     * PREPARE_<h>_<side> -> READY_<h>_<side> when At Position
     * READY_<h>_<side> -> SCORE_<h>_<side> when External Signal
     */
    PREPARE_L4_LEFT(CORAL),
    PREPARE_L3_LEFT(CORAL),
    PREPARE_L2_LEFT(CORAL),
    READY_L4_LEFT(CORAL),
    READY_L3_LEFT(CORAL),
    READY_L2_LEFT(CORAL),
    SCORE_L4_LEFT(CORAL),
    SCORE_L3_LEFT(CORAL),
    SCORE_L2_LEFT(CORAL),

    PREPARE_L4_RIGHT(CORAL),
    PREPARE_L3_RIGHT(CORAL),
    PREPARE_L2_RIGHT(CORAL),
    READY_L4_RIGHT(CORAL),
    READY_L3_RIGHT(CORAL),
    READY_L2_RIGHT(CORAL),
    SCORE_L4_RIGHT(CORAL),
    SCORE_L3_RIGHT(CORAL),
    SCORE_L2_RIGHT(CORAL),


    /*
     * ALGAE INTAKE STATES
     *
     * PREPARE_INTAKE_<lvl>_REEF_ALGAE_<side> -> INTAKE_<lvl>_REEF_ALGAE_<side> when At Position
     * INTAKE_<lvl>_REEF_ALGAE_<side> -> IDLE_ALGAE when Algae Detected and Away from Reef
     *
     * PREPARE_INTAKE_GROUND_ALGAE -> INTAKE_GROUND_ALGAE when At Position
     * INTAKE_GROUND_ALGAE -> IDLE_ALGAE when External Signal
     */
    PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT(ALGAE),
    PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT(ALGAE),
    PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT(ALGAE),
    PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT(ALGAE),
    ACTIVE_INTAKE_HIGH_REEF_ALGAE_LEFT(ALGAE),
    ACTIVE_INTAKE_HIGH_REEF_ALGAE_RIGHT(ALGAE),
    ACTIVE_INTAKE_LOW_REEF_ALGAE_LEFT(ALGAE),
    ACTIVE_INTAKE_LOW_REEF_ALGAE_RIGHT(ALGAE),

    PREPARE_INTAKE_GROUND_ALGAE(ALGAE),
    ACTIVE_INTAKE_GROUND_ALGAE(ALGAE),


    /*
     * ALGAE SCORE STATES
     *
     * PREPARE_SCORE_ALGAE_NET_<side> -> READY_SCORE_ALGAE_NET_<side> when At Position
     * READY_SCORE_ALGAE_NET_<side> -> SCORE_ALGAE_NET_<side> when External Signal
     * SCORE_ALGAE_NET_<side> -> IDLE_ALGAE when Slight Delay
     *
     * PREPARE_SCORE_ALGAE_PROCESSOR -> READY_SCORE_ALGAE_PROCESSOR when At Position
     * READY_SCORE_ALGAE_PROCESSOR -> SCORE_ALGAE_PROCESSOR when External Signal
     * SCORE_ALGAE_PROCESSOR -> IDLE_ALGAE when Slight Delay
     */
    PREPARE_SCORE_ALGAE_NET_LEFT(ALGAE),
    PREPARE_SCORE_ALGAE_NET_RIGHT(ALGAE),
    PREPARE_SCORE_ALGAE_PROCESSOR(ALGAE),
    READY_SCORE_ALGAE_NET_LEFT(ALGAE),
    READY_SCORE_ALGAE_NET_RIGHT(ALGAE),
    READY_SCORE_ALGAE_PROCESSOR(ALGAE),
    SCORE_ALGAE_NET_LEFT(ALGAE),
    SCORE_ALGAE_NET_RIGHT(ALGAE),
    SCORE_ALGAE_PROCESSOR(ALGAE),

    /*
     * LOLLIPOP STATES
     *
     * PREPARE_INTAKE_LOLLIPOP -> ACTIVE_INTAKE_LOLLIPOP when At Position
     * ACTIVE_INTAKE_LOLLIPOP -> IDLE_CORAL when External Signal
     */
    PREPARE_INTAKE_LOLLIPOP(CORAL),
    ACTIVE_INTAKE_LOLLIPOP(CORAL),

    /*
     * CLIMB STATES
     *
     * PREPARE_CLIMB when External Signal
     * PREPARE_CLIMB -> READY_CLIMB when At Position
     */
    PREPARE_CLIMB(NONE),
    READY_CLIMB(NONE);

    public enum HandGamePieceState {
        CORAL, ALGAE, NONE;
        public boolean isCoral() {
            return this == CORAL;
        }
        public boolean isAlgae() {
            return this == ALGAE;
        }
        public boolean isNone() {
            return this == NONE;
        }
    }

    public final HandGamePieceState handGamePieceState;

    ArmManagerState(HandGamePieceState handGamePieceState) {
        this.handGamePieceState = handGamePieceState;
    }

    /* ******** Idle Utilities ******** */
    public boolean isIdleState() {
        return switch (this) {
            case IDLE_ALGAE, IDLE_CORAL, IDLE_EMPTY -> true;
            default -> false;
        };
    }

    public static ArmManagerState getIdleStateFor(HandGamePieceState handGamePieceState) {
        return switch (handGamePieceState) {
            case CORAL -> IDLE_CORAL;
            case ALGAE -> IDLE_ALGAE;
            case NONE -> IDLE_EMPTY;
        };
    }


    /* ******** Handoff Utilities ******** */

    public boolean isHandoffReadyState() {
        return switch (this) {
            case READY_HANDOFF_LEFT, READY_HANDOFF_MIDDLE, READY_HANDOFF_RIGHT -> true;
            default -> false;
        };
    }

    /**
     * Get the "IDLE_XXX" state based on the given "PREPARE_IDLE_XXX" state.
     * Returns the given state if this is not a idle state.
     */
    public ArmManagerState getPrepareToIdleState() {
        return switch (this) {
            case PREPARE_IDLE_EMPTY -> IDLE_EMPTY;
            case PREPARE_IDLE_ALGAE -> IDLE_ALGAE;
            case PREPARE_IDLE_CORAL -> IDLE_CORAL;
            default -> this;
        };
    }


    /**
     * Get the "READY_HANDOFF_XXX" state based on the given "PREPARE_HANDOFF_XXX" state.
     * Returns the given state if this is not a handoff state.
     */
    public ArmManagerState getHandoffPrepareToReadyState() {
        return switch (this) {
            case PREPARE_HANDOFF_LEFT -> READY_HANDOFF_LEFT;
            case PREPARE_HANDOFF_MIDDLE -> READY_HANDOFF_MIDDLE;
            case PREPARE_HANDOFF_RIGHT -> READY_HANDOFF_RIGHT;
            default -> this;
        };
    }


    /**
     * Get the "EXECUTE_HANDOFF_XXX" state based on the given "READY_HANDOFF_XXX" state.
     * Returns the given state if this is not a handoff state.
     */
    public ArmManagerState getHandoffReadyToExecuteState() {
        return switch (this) {
            case READY_HANDOFF_LEFT -> EXECUTE_HANDOFF_LEFT;
            case READY_HANDOFF_MIDDLE -> EXECUTE_HANDOFF_MIDDLE;
            case READY_HANDOFF_RIGHT -> EXECUTE_HANDOFF_RIGHT;
            default -> this;
        };
    }

    /**
     * Get the "PREEMPTIVE_HANDOFF_XXX" state based on the given CoralDetectorState.
     * Returns PREEMPTIVE_HANDOFF_MIDDLE if NONE is passed.
     */
    public static ArmManagerState getHandoffPreemptiveFromCoralPosition(CoralDetectorState coralPosition) {
        return switch (coralPosition) {
            case LEFT -> PREEMPTIVE_HANDOFF_LEFT;
            case MIDDLE -> PREEMPTIVE_HANDOFF_MIDDLE;
            case RIGHT -> PREEMPTIVE_HANDOFF_RIGHT;
            default -> PREEMPTIVE_HANDOFF_MIDDLE;
        };
    }

    /**
     * Get the "PREPARE_HANDOFF_XXX" state based on the given CoralDetectorState.
     * Returns PREPARE_HANDOFF_MIDDLE if NONE is passed.
     */
    public static ArmManagerState getHandoffPrepareFromCoralPosition(CoralDetectorState coralPosition) {
        return switch (coralPosition) {
            case LEFT -> PREPARE_HANDOFF_LEFT;
            case MIDDLE -> PREPARE_HANDOFF_MIDDLE;
            case RIGHT -> PREPARE_HANDOFF_RIGHT;
            default -> PREPARE_HANDOFF_MIDDLE;
        };
    }


    /* ******** Coral Score Utilities ******** */
    public boolean isCoralReadyToScoreState() {
        return switch (this) {
            case READY_L4_LEFT, READY_L3_LEFT, READY_L2_LEFT, READY_L4_RIGHT, READY_L3_RIGHT, READY_L2_RIGHT -> true;
            default -> false;
        };
    }

    public boolean isCoralScoreState() {
        return switch (this) {
            case SCORE_L4_LEFT, SCORE_L3_LEFT, SCORE_L2_LEFT, SCORE_L4_RIGHT, SCORE_L3_RIGHT, SCORE_L2_RIGHT -> true;
            default -> false;
        };
    }

    /**
     * Get the "PREPARE_XXX_XXX" state based on the given robot side and scoring level.
     */
    public static ArmManagerState getCoralPrepareScore(RobotScoringSide robotSide, FieldConstants.PipeScoringLevel scoringLevel) {
        return switch (scoringLevel) {
            case L4 -> robotSide == RobotScoringSide.LEFT ? PREPARE_L4_LEFT : PREPARE_L4_RIGHT;
            case L3 -> robotSide == RobotScoringSide.LEFT ? PREPARE_L3_LEFT : PREPARE_L3_RIGHT;
            case L2 -> robotSide == RobotScoringSide.LEFT ? PREPARE_L2_LEFT : PREPARE_L2_RIGHT;
        };
    }

    /**
     * Get the "READY_XXX_XXX" state based on the given "PREPARE_XXX_XXX" state.
     * Returns the given state if this is not a coral prepare state.
     */
    public ArmManagerState getCoralPrepareToReadyState() {
        return switch (this) {
            case PREPARE_L4_LEFT -> READY_L4_LEFT;
            case PREPARE_L3_LEFT -> READY_L3_LEFT;
            case PREPARE_L2_LEFT -> READY_L2_LEFT;
            case PREPARE_L4_RIGHT -> READY_L4_RIGHT;
            case PREPARE_L3_RIGHT -> READY_L3_RIGHT;
            case PREPARE_L2_RIGHT -> READY_L2_RIGHT;
            default -> this;
        };
    }

    /**
     * Get the "SCORE_XXX_XXX" state based on the given "READY_XXX_XXX" state.
     * Returns the given state if this is not a coral ready state.
     */
    public ArmManagerState getCoralReadyToScoreState() {
        return switch (this) {
            case READY_L4_LEFT -> SCORE_L4_LEFT;
            case READY_L3_LEFT -> SCORE_L3_LEFT;
            case READY_L2_LEFT -> SCORE_L2_LEFT;
            case READY_L4_RIGHT -> SCORE_L4_RIGHT;
            case READY_L3_RIGHT -> SCORE_L3_RIGHT;
            case READY_L2_RIGHT -> SCORE_L2_RIGHT;
            default -> this;
        };
    }


    /* ******** Algae Intake Utilities ******** */

    /**
     * Get the "ACTIVE_INTAKE_XXX" state based on the given "PREPARE_INTAKE_XXX" state.
     * Returns the given state if this is not a algae intake prepare state.
     */
    public ArmManagerState getAlgaeIntakePrepareToActiveState() {
        return switch (this) {
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT -> ACTIVE_INTAKE_HIGH_REEF_ALGAE_LEFT;
            case PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT -> ACTIVE_INTAKE_HIGH_REEF_ALGAE_RIGHT;
            case PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT -> ACTIVE_INTAKE_LOW_REEF_ALGAE_LEFT;
            case PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT -> ACTIVE_INTAKE_LOW_REEF_ALGAE_RIGHT;
            case PREPARE_INTAKE_GROUND_ALGAE -> ACTIVE_INTAKE_GROUND_ALGAE;
            default -> this;
        };
    }

    /**
     * Get the "PREPARE_INTAKE_XXX_XXX" state based on the given robot side and level.
     */
    public static ArmManagerState getAlgaeIntakePrepare(RobotScoringSide robotSide, boolean top) {
        return switch (robotSide) {
            case LEFT -> top ? PREPARE_INTAKE_HIGH_REEF_ALGAE_LEFT : PREPARE_INTAKE_LOW_REEF_ALGAE_LEFT;
            case RIGHT -> top ? PREPARE_INTAKE_HIGH_REEF_ALGAE_RIGHT : PREPARE_INTAKE_LOW_REEF_ALGAE_RIGHT;
        };
    }


    /* ******** Algae Score Utilities ******** */
    public boolean isNetReadyState() {
        return switch (this) {
            case READY_SCORE_ALGAE_NET_LEFT, READY_SCORE_ALGAE_NET_RIGHT -> true;
            default -> false;
        };
    }

    /**
     * Get the "READY_SCORE_ALGAE_XXX" state based on the given "PREPARE_SCORE_ALGAE_XXX" state.
     * Returns the given state if this is not a algae score prepare state.
     */
    public ArmManagerState getAlgaeNetPrepareToReadyState() {
        return switch (this) {
            case PREPARE_SCORE_ALGAE_NET_LEFT -> READY_SCORE_ALGAE_NET_LEFT;
            case PREPARE_SCORE_ALGAE_NET_RIGHT -> READY_SCORE_ALGAE_NET_RIGHT;
            default -> this;
        };
    }

    /**
     * Get the "SCORE_ALGAE_XXX" state based on the given "READY_SCORE_ALGAE_XXX" state.
     * Returns the given state if this is not a algae score ready state.
     */
    public ArmManagerState getAlgaeNetReadyToScoreState() {
        return switch (this) {
            case READY_SCORE_ALGAE_NET_LEFT -> SCORE_ALGAE_NET_LEFT;
            case READY_SCORE_ALGAE_NET_RIGHT -> SCORE_ALGAE_NET_RIGHT;
            default -> this;
        };
    }

    /**
     * Get the "PREPARE_SCORE_ALGAE_NET_XXX" state based on the given robot side.
     */
    public static ArmManagerState getNetPrepareScore(RobotScoringSide robotSide) {
        return robotSide == RobotScoringSide.LEFT ? PREPARE_SCORE_ALGAE_NET_LEFT : PREPARE_SCORE_ALGAE_NET_RIGHT;
    }
}
