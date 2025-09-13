package frc.robot.subsystems.armManager.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class SimElevator {
    private static ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60Foc(2), 
        3.5, Units.lbsToKilograms(13), Units.inchesToMeters(1), 0, 1.4, true, 0, 0.0001, 0);
    
    public static void updateSimPosition(TalonFX left, TalonFX right) {
        var leftSim = left.getSimState();
        var rightSim = right.getSimState();
        leftSim.Orientation = ChassisReference.Clockwise_Positive;
        rightSim.Orientation = ChassisReference.CounterClockwise_Positive;
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = leftSim.getMotorVoltageMeasure();
        elevatorSim.setInputVoltage(motorVoltage.in(Volts));
        elevatorSim.update(Constants.SIM_LOOP_TIME);
        leftSim.setRawRotorPosition(elevatorSim.getPositionMeters() * ElevatorConstants.ElevatorGearRatio);
        leftSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.ElevatorGearRatio);
        rightSim.setRawRotorPosition(elevatorSim.getPositionMeters() * ElevatorConstants.ElevatorGearRatio);
        rightSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.ElevatorGearRatio);
    }
}
