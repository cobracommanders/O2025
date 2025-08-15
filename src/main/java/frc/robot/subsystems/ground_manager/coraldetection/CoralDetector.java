package frc.robot.subsystems.ground_manager.coraldetection;

import com.ctre.phoenix6.hardware.CANrange;

import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.armManager.ArmManagerStates;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorStates;

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
        if (lDetected && rDetected) {
            return CoralDetectorStates.MIDDLE;
        } else if (!lDetected && !rDetected) {
            return CoralDetectorStates.NONE;
        } else if (lDetected && !rDetected) {
            return CoralDetectorStates.LEFT;
        } else {
            return CoralDetectorStates.RIGHT;
        }

    }

    private static CoralDetector instance;

    public static CoralDetector getInstance() {
        if (instance == null)
            instance = new CoralDetector();
        return instance;
    }
}
// Would we want to log this stuff? If yes, what would we log?
// Would we want an after transitions? and could i get an explaination on what
// it is cause i saw it and i don't exactly know what it is?
// I was thinking about arm and handoff and such but i would like to get
// assistance on it - if its included on this subsystem!
