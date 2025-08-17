package frc.robot;

public class Ports {
    public static class IntakePivotPorts {
        public static final int INTAKE_MOTOR = 20; //to do: add values
        public static final int INTAKE_PIVOT_DUTY_CYCLE_ENCODER = 0; //this is plugged into DIO
    }

    public static class IntakeRollersPorts {
        public static final int INTAKE_ROLLER_MOTOR = 21; //to do: add values
    }

    public static class coralDetectorPorts{
        public static final int LEFT_CAN_RANGE = 22;
        public static final int RIGHT_CAN_RANGE = 23;
    }

    public static class ClimberPorts {
        public static int WHEEL_CLIMBER_MOTOR_PORT = 31;
        public static int WINCH_CLIMBER_MOTOR_PORT = 30;
        public static int CLIMER_DUTY_CYCLE_ENCODER = 1; //this is plugged into DIO
    }
    
    public static class ArmPorts {
        public static final int MOTOR = 40; //22
        public static final int ENCODER = 41; //0
    }

    public static class ElevatorPorts {
        public static final int LMOTOR = 42; //22
        public static final int RMOTOR = 43;
    }

    public static class HandPorts {
        public static final int MOTOR = 44;
    }
}
