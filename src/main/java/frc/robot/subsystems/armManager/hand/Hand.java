package frc.robot.subsystems.armManager.hand;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.util.PhoenixSignalManager;

import static java.lang.Math.abs;

public class Hand extends StateMachine<HandState> {
    private final TalonFX motor = new TalonFX(Ports.HandPorts.MOTOR, TunerConstants.kCANBus.getName());

    private double statorCurrent;
    private double angularVelocity;

    private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();

    public Hand() {
        super(HandState.IDLE_CORAL, "Hand");
        TalonFXConfiguration motor_config = new TalonFXConfiguration();
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motor_config);

        PhoenixSignalManager.registerSignals(true, statorCurrentSignal, velocitySignal);
    }

    @Override
    public void collectInputs() {
        statorCurrent = statorCurrentSignal.getValueAsDouble();
        angularVelocity = velocitySignal.getValueAsDouble();
        DogLog.log("Hand/Stator Current", statorCurrent);
        DogLog.log("Hand/Angular Velocity", angularVelocity);
//        DogLog.log("Hand/hasAlgaeForIntake", hasAlgaeForIntake());
    }

    public void setState(HandState state) {
        setStateFromRequest(state);
    }

    private final Debouncer hasAlgaeDebouncer = new Debouncer(0.25);

    public boolean hasAlgaeForIntake() {
        return hasAlgaeDebouncer.calculate(abs(angularVelocity) < 55);
    }

    public boolean droppedAlgae() {
        return abs(angularVelocity) > 55;
    }

    @Override
    protected void afterTransition(HandState newState) {
        // Custom cases can go here, default to regular speed control
        switch (newState) {
            default -> {
                double speed = newState.getSpeed();
                DogLog.log("Hand/Speed", speed);
                motor.set(speed);
            }
        }
    }
}
