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
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.mechanism_visualizer.MechanismVisualizer;
import frc.robot.stateMachine.StateMachine;

public class Elevator extends StateMachine<ElevatorState> {
    public final DoubleSubscriber elevatorSpeed = DogLog.tunable("elevator/Speed [-1, 1]", 0.0);
    private final String name = getName();
    public static TalonFX lMotor;
    public static TalonFX rMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I)
                    .withKD(ElevatorConstants.D).withKG(ElevatorConstants.G)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.ElevatorGearRatio));

    private double elevatorPosition;
    public final double tolerance;
    private final Follower right_motor_request = new Follower(Ports.ElevatorPorts.LMOTOR, true);
    private final MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);

    public Elevator() {
        super(ElevatorState.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;

        lMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
        rMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);

        lMotor.getConfigurator().apply(motor_config);
        rMotor.getConfigurator().apply(motor_config);

        lMotor.setPosition(0);
        rMotor.setPosition(0);

        tolerance = 0.015;
    }

    protected ElevatorState getNexState(ElevatorState currentState) {
        return currentState;
    }

    public boolean atGoal() {
        return switch (getState()) {
            case LOLLIPOP -> MathUtil.isNear(ElevatorPosition.LOLLIPOP, elevatorPosition, tolerance);
            case IDLE -> MathUtil.isNear(ElevatorPosition.IDLE, elevatorPosition, tolerance);
            case L4 -> MathUtil.isNear(ElevatorPosition.L4, elevatorPosition, tolerance);
            case HIGH_REEF_ALGAE -> MathUtil.isNear(ElevatorPosition.HIGH_REEF_ALGAE, elevatorPosition, tolerance);
            case LOW_REEF_ALGAE -> MathUtil.isNear(ElevatorPosition.LOW_REEF_ALGAE, elevatorPosition, tolerance);
            case GROUND_ALGAE -> MathUtil.isNear(ElevatorPosition.GROUND_ALGAE, elevatorPosition, tolerance);
            case ALGAE_NET -> MathUtil.isNear(ElevatorPosition.ALGAE_NET, elevatorPosition, tolerance);
            case ALGAE_PROCESSOR -> MathUtil.isNear(ElevatorPosition.ALGAE_PROCESSOR, elevatorPosition, tolerance);
            case HANDOFF -> MathUtil.isNear(ElevatorPosition.HANDOFF, elevatorPosition, tolerance);
            case SCORE_L4 -> MathUtil.isNear(ElevatorPosition.SCORE_L4, elevatorPosition, tolerance);
            case L3 -> MathUtil.isNear(ElevatorPosition.L3, elevatorPosition, tolerance);
            case SCORE_L3 -> MathUtil.isNear(ElevatorPosition.SCORE_L3, elevatorPosition, tolerance);
            case L2 -> MathUtil.isNear(ElevatorPosition.L2, elevatorPosition, tolerance);
            case SCORE_L2 -> MathUtil.isNear(ElevatorPosition.SCORE_L2, elevatorPosition, tolerance);
            case HANDOFF_CORAL_MODE ->
                    MathUtil.isNear(ElevatorPosition.HANDOFF_CORAL_MODE, elevatorPosition, tolerance);
        };

    }

    @Override
    public void collectInputs() {
        elevatorPosition = lMotor.getPosition().getValueAsDouble();
        double leftElevatorPosition = elevatorPosition;
        double rightElevatorPosition = rMotor.getPosition().getValueAsDouble();
        DogLog.log(name + "/Left Elevator Position", leftElevatorPosition);
        DogLog.log(name + "/Right Elevator Position", rightElevatorPosition);
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

    public void setElevatorSpeed() {
        lMotor.set(elevatorSpeed.get());
        rMotor.set(-elevatorSpeed.get());
    }

    public void setElevatorPosition(double position) {
        DogLog.log(name + "/Setpoint", position);
        rMotor.setControl(right_motor_request);
        lMotor.setControl(motor_request.withPosition(position));
    }

    @Override
    protected void afterTransition(ElevatorState newState) {
        switch (newState) {
            case LOLLIPOP -> {
                setElevatorPosition(ElevatorPosition.LOLLIPOP);
            }
            case IDLE -> {
                setElevatorPosition(ElevatorPosition.IDLE);
            }
            case HANDOFF_CORAL_MODE -> {
                setElevatorPosition(ElevatorPosition.HANDOFF_CORAL_MODE);
            }
            case L4 -> {
                setElevatorPosition(ElevatorPosition.L4);
            }
            case HIGH_REEF_ALGAE -> {
                setElevatorPosition(ElevatorPosition.HIGH_REEF_ALGAE);
            }
            case LOW_REEF_ALGAE -> {
                setElevatorPosition(ElevatorPosition.LOW_REEF_ALGAE);
            }
            case GROUND_ALGAE -> {
                setElevatorPosition(ElevatorPosition.GROUND_ALGAE);
            }
            case ALGAE_PROCESSOR -> {
                setElevatorPosition(ElevatorPosition.ALGAE_PROCESSOR);
            }
            case ALGAE_NET -> {
                setElevatorPosition(ElevatorPosition.ALGAE_NET);
            }
            case HANDOFF -> {
                setElevatorPosition(ElevatorPosition.HANDOFF);
            }
            case SCORE_L4 -> {
                setElevatorPosition(ElevatorPosition.SCORE_L4);
            }
            case L3 -> {
                setElevatorPosition(ElevatorPosition.L3);
            }
            case SCORE_L3 -> {
                setElevatorPosition(ElevatorPosition.SCORE_L3);
            }
            case L2 -> {
                setElevatorPosition(ElevatorPosition.L2);
            }
            case SCORE_L2 -> {
                setElevatorPosition(ElevatorPosition.SCORE_L2);
            }
        }
    }

    public void tickUp() {
        switch (getState()) {
            case L4:
                ElevatorPosition.L4 += .015;
                setElevatorPosition(ElevatorPosition.L4);
                break;

            case L3:
                ElevatorPosition.L3 += .015;
                setElevatorPosition(ElevatorPosition.L3);
                break;

            case L2:
                ElevatorPosition.L2 += .015;
                setElevatorPosition(ElevatorPosition.L2);
                break;

            case ALGAE_NET:
                ElevatorPosition.ALGAE_NET += .015;
                setElevatorPosition(ElevatorPosition.ALGAE_NET);
                break;

            case ALGAE_PROCESSOR:
                ElevatorPosition.ALGAE_PROCESSOR += .015;
                setElevatorPosition(ElevatorPosition.ALGAE_PROCESSOR);
                break;

            case HIGH_REEF_ALGAE:
                ElevatorPosition.HIGH_REEF_ALGAE += .015;
                setElevatorPosition(ElevatorPosition.HIGH_REEF_ALGAE);
                break;

            case LOW_REEF_ALGAE:
                ElevatorPosition.LOW_REEF_ALGAE += .015;
                setElevatorPosition(ElevatorPosition.LOW_REEF_ALGAE);
                break;

            case GROUND_ALGAE:
                ElevatorPosition.GROUND_ALGAE += .015;
                setElevatorPosition(ElevatorPosition.GROUND_ALGAE);
                break;

        }
    }

    public void tickDown() {
        switch (getState()) {
            case L4:
                ElevatorPosition.L4 -= .015;
                setElevatorPosition(ElevatorPosition.L4);
                break;

            case L3:
                ElevatorPosition.L3 -= .015;
                setElevatorPosition(ElevatorPosition.L3);
                break;

            case L2:
                ElevatorPosition.L2 -= .015;
                setElevatorPosition(ElevatorPosition.L2);
                break;

            case ALGAE_NET:
                ElevatorPosition.ALGAE_NET -= .015;
                setElevatorPosition(ElevatorPosition.ALGAE_NET);
                break;

            case ALGAE_PROCESSOR:
                ElevatorPosition.ALGAE_PROCESSOR -= .015;
                setElevatorPosition(ElevatorPosition.ALGAE_PROCESSOR);
                break;

            case HIGH_REEF_ALGAE:
                ElevatorPosition.HIGH_REEF_ALGAE -= .015;
                setElevatorPosition(ElevatorPosition.HIGH_REEF_ALGAE);
                break;

            case LOW_REEF_ALGAE:
                ElevatorPosition.LOW_REEF_ALGAE -= .015;
                setElevatorPosition(ElevatorPosition.LOW_REEF_ALGAE);
                break;

            case GROUND_ALGAE:
                ElevatorPosition.GROUND_ALGAE -= .015;
                setElevatorPosition(ElevatorPosition.GROUND_ALGAE);
                break;
        }
    }
}
