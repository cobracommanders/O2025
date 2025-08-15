package frc.robot.subsystems.ground_manager.coraldetection;

import com.ctre.phoenix6.hardware.CANrange;

import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class CoralDetector extends StateMachine<CoralDetectorStates> {
    public CANrange lCANRange;
    public CANrange rCANRange;
    public boolean lDetected = false;
    public boolean rDetected = false;

    public CoralDetector() {
        super(CoralDetectorStates.NONE);
        lCANRange = new CANrange(Ports.coralDetectorPorts.LEFT_CAN_RANGE);
        rCANRange = new CANrange(Ports.coralDetectorPorts.RIGHT_CAN_RANGE);
    }

    protected void collectInputs() {
        lDetected = lCANRange.getIsDetected().getValue();
        rDetected = rCANRange.getIsDetected().getValue();
    }

    @Override
    protected CoralDetectorStates getNextState(CoralDetectorStates currentState) {
    }

}
