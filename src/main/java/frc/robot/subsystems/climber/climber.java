// todo: rename THIS FILE.
package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.stateMachine.StateMachine;

public class Climber extends StateMachine<ClimberStates>{
    private final TalonFX lMotor;
    private final TalonFX rMotor;

    public Climber(){
      super();
    }

}
