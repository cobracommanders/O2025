package frc.robot.subsystems.ground_manager.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakePivotConstants;

public class SimPivot {
    private static final double intakeLength = Units.inchesToMeters(12); // m
    private static final double intakeMass = Units.lbsToKilograms(7);
    private static final SingleJointedArmSim armSim =
            new SingleJointedArmSim(
                    DCMotor.getKrakenX60Foc(1),
                    IntakePivotConstants.PivotGearRatio,
                    SingleJointedArmSim.estimateMOI(intakeLength, intakeMass),
                    intakeLength,
                    Units.rotationsToRadians(IntakePivotPositions.INTAKING),
                    Units.rotationsToRadians(IntakePivotPositions.HANDOFF),
                    true,
                    Units.rotationsToRadians(IntakePivotPositions.IDLE),
                    0.001,
                    0);

    public static void updateSimPosition(TalonFX motor) {
        var motorSim = motor.getSimState();
        motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = motorSim.getMotorVoltageMeasure();
        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(Constants.SIM_LOOP_TIME);
        motorSim.setRawRotorPosition(
                Units.radiansToRotations(armSim.getAngleRads())
                        * IntakePivotConstants.PivotGearRatio);
        motorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec())
                        * IntakePivotConstants.PivotGearRatio);
    }
}
