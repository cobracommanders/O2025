package frc.robot.subsystems.ground_manager.coraldetection;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import dev.doglog.DogLog;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.PhoenixSignalManager;

public class CoralDetector extends StateMachine<CoralDetectorState> {
    private final CANrange lCANRange = new CANrange(Ports.coralDetectorPorts.LEFT_CAN_RANGE);
    private final CANrange rCANRange = new CANrange(Ports.coralDetectorPorts.RIGHT_CAN_RANGE);

    public boolean lDetected = false;
    public boolean rDetected = false;
    public double rDistance;

    public CoralDetectorState simCoralPosition = CoralDetectorState.NONE;

    private final StatusSignal<Boolean> lIsDetectedSignal = lCANRange.getIsDetected();
    private final StatusSignal<Boolean> rIsDetectedSignal = rCANRange.getIsDetected();

    private CoralDetector() {
        super(CoralDetectorState.NONE, "CoralDetector");

        CANrangeConfiguration config = new CANrangeConfiguration();

        config.ProximityParams.ProximityHysteresis = 0.008;
        config.ProximityParams.ProximityThreshold = 0.065;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 8000;

        lCANRange.getConfigurator().apply(config);
        rCANRange.getConfigurator().apply(config);

        PhoenixSignalManager.registerSignals(false, lIsDetectedSignal, rIsDetectedSignal);
    }

    protected void collectInputs() {

        //We can switch to using .isDetected() if we would like.

        lDetected = lIsDetectedSignal.getValue();
        rDetected = rIsDetectedSignal.getValue();

        DogLog.log(name + "/Left Detected", lDetected);
        DogLog.log(name + "/Right Detected", rDetected);

        // DogLog.log(name + "/Left Signal", lCANRange.getSignalStrength().getValueAsDouble());
        // DogLog.log(name + "/Right Signal", rCANRange.getSignalStrength().getValueAsDouble());
    }

    @Override
    protected CoralDetectorState getNextState(CoralDetectorState currentState) {
        if (Utils.isSimulation()) {
            return simCoralPosition;
        }
        if (lDetected && rDetected) {
            return CoralDetectorState.MIDDLE;
        } else if (!lDetected && !rDetected) {
            return CoralDetectorState.NONE;
        } else if (lDetected && !rDetected) {
            return CoralDetectorState.LEFT;
        } else {
            return CoralDetectorState.RIGHT;
        }

    }

    public boolean hasCoral() {
        return getState() != CoralDetectorState.NONE;
    }

    public void setSimCoral(CoralDetectorState position) {
        this.simCoralPosition = position;
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
