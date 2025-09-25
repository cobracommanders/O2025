package frc.robot.stateMachine;

/*
 * Naming Convention is: ACTION_LOCATION
 */

public enum RequestManagerState {
    INDEPENDENT,
    PREPARE_HANDOFF_GROUND,
    PREPARE_HANDOFF_ARM,
    HANDOFF,
    PREPARE_INVERTED_HANDOFF,
    INVERTED_HANDOFF,
    CLIMB,
    PREPARE_IDLE
}
