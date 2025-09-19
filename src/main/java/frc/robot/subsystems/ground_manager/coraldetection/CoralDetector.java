package frc.robot.subsystems.ground_manager.coraldetection;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import dev.doglog.DogLog;
import frc.robot.Constants;

import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.ground_manager.coraldetection.CoralDetectorStates;

public class CoralDetector extends StateMachine<CoralDetectorStates> {
    public CANrange lCANRange;
    public CANrange rCANRange;
    public boolean lDetected = false;
    public boolean rDetected = false;
    public double lDistance;
    public double rDistance;
    public final String name;
    
    public boolean hasSimCoral = false;

    public CoralDetector() {
        super(CoralDetectorStates.NONE);
        lCANRange = new CANrange(Ports.coralDetectorPorts.LEFT_CAN_RANGE);
        rCANRange = new CANrange(Ports.coralDetectorPorts.RIGHT_CAN_RANGE);
        this.name = getName();
    }

    protected void collectInputs() {
        lDistance = lCANRange.getDistance().getValueAsDouble();
        rDistance = rCANRange.getDistance().getValueAsDouble();
        
        //We can switch to using .isDetected() if we would like.
        lDetected = lDistance < Constants.CoralDetectorConstants.DETECTION_THRESHOLD;
        rDetected = rDistance < Constants.CoralDetectorConstants.DETECTION_THRESHOLD;
        DogLog.log(name + "/Left Detected", lDetected);
        DogLog.log(name + "/Right Detected", rDetected);
        DogLog.log(name + "/Left Distance", lDistance);
        DogLog.log(name + "/Right Distance", rDistance);
    }

    @Override
    protected CoralDetectorStates getNextState(CoralDetectorStates currentState) {
        if (Utils.isSimulation()){
            if (hasSimCoral){
                return CoralDetectorStates.MIDDLE;
            } else {
                return CoralDetectorStates.NONE;
            }
        }
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

    public boolean hasCoral(){
        return getState() != CoralDetectorStates.NONE;
    }

    public void setSimCoral(boolean hasCoral){
        this.hasSimCoral = hasCoral;
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
