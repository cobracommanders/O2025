package frc.robot.stateMachine;

/*
 * Naming Convention is: ACTION_LOCATION
 */

public enum RequestManagerState {
    INDEPENDENT,
    PREPARE_HANDOFF,
    HANDOFF,
    PREPARE_INVERTED_HANDOFF,
    INVERTED_HANDOFF,
    CLIMB,
    PREPARE_IDLE
}
