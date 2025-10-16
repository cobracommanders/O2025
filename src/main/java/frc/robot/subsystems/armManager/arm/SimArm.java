package frc.robot.subsystems.armManager.arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class SimArm {
    private static final double armLength = 0.7; // m
    private static final double armMass = Units.lbsToKilograms(7);
    private static final SingleJointedArmSim armSim =
            new SingleJointedArmSim(
                    DCMotor.getKrakenX60Foc(1),
                    ArmConstants.ArmGearRatio,
                    SingleJointedArmSim.estimateMOI(armLength, armMass),
                    armLength,
                    -1000000000.0,
                    1000000000.0,
                    true,
                    Units.rotationsToRadians(ArmState.START_POSITION.getPosition()),
                    0.001,
                    0);

    public static void updateSimPosition(TalonFX motor, CANcoder encoder) {
        var encoderSim = encoder.getSimState();
        var motorSim = motor.getSimState();
        encoderSim.Orientation = ChassisReference.Clockwise_Positive;
        motorSim.Orientation = ChassisReference.Clockwise_Positive;
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = motorSim.getMotorVoltageMeasure();
        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(Constants.SIM_LOOP_TIME);
        motorSim.setRawRotorPosition(
                Units.radiansToRotations(armSim.getAngleRads()) * ArmConstants.ArmGearRatio);
        motorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec())
                        * ArmConstants.ArmGearRatio);
        encoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()));
    }
}
