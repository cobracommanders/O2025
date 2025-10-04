package frc.robot.subsystems.armManager.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.StateMachine;

public class Elevator extends StateMachine<ElevatorState> {
    private static TalonFX lMotor;
    private static TalonFX rMotor;

    private double elevatorPosition;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    public Elevator() {
        super(ElevatorState.IDLE);
        TalonFXConfiguration motor_config = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ElevatorConstants.P)
                                .withKI(ElevatorConstants.I)
                                .withKD(ElevatorConstants.D)
                                .withKG(ElevatorConstants.G)
                                .withGravityType(GravityTypeValue.Elevator_Static)
                )
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.ElevatorGearRatio));
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;

        lMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
        rMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);

        lMotor.getConfigurator().apply(motor_config);
        rMotor.getConfigurator().apply(motor_config);

        lMotor.setPosition(0);
        rMotor.setPosition(0);

        rMotor.setControl(new Follower(Ports.ElevatorPorts.LMOTOR, true));
    }

    public boolean atGoal() {
        return MathUtil.isNear(getState().getPosition(), elevatorPosition, ElevatorConstants.Tolerance);
    }

    @Override
    public void collectInputs() {
        elevatorPosition = lMotor.getPosition().getValueAsDouble();
        DogLog.log("Elevator/Position", elevatorPosition);
        DogLog.log("Elevator/At Goal", atGoal());
        MechanismVisualizer.setElevatorPosition(elevatorPosition);
    }

    @Override
    public void simulationPeriodic() {
        SimElevator.updateSimPosition(lMotor, rMotor);
    }

    public double getHeight() {
        return elevatorPosition;
    }

    public void setState(ElevatorState state) {
        setStateFromRequest(state);
    }

    @Override
    public ElevatorState getState() {
        return super.getState();
    }

    @Override
    protected void afterTransition(ElevatorState newState) {
        switch (newState) {
            // Custom cases can go here, default to standard position control
            default -> {
                double position = newState.getPosition();
                DogLog.log("Elevator/Setpoint", position);
                lMotor.setControl(motionMagicVoltage.withPosition(position));
            }
        }
    }
}
