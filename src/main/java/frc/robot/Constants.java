package frc.robot;

import frc.robot.subsystems.armManager.elevator.ElevatorState;
import frc.robot.subsystems.drivetrain.TunerConstants;

public final class Constants {

    public static boolean tuningMode = true;

    public static boolean isSimulation = false;

    public static final double SIM_LOOP_TIME = 0.02;

    public static final class ArmConstants {
        public static final double ArmGearRatio = 50.2232;
        public static final double P = 168;
        public static final double I = 0;
        public static final double D = 0;
        public static final double G = 0.24;
        public static final double MotionMagicCruiseVelocity = 7.5;
        public static final double MotionMagicAcceleration = 3.25;
//        public static final double encoderOffset = -0.64111328125 + 0.005371;
        public static final double encoderOffset = -0.384277 - 0.25;
        public static final double inchesFromCenter = 6.615;
        public static final double Tolerance = 0.01;
    }

    public static final class ElevatorConstants {
        public static final double ElevatorGearRatio = 21.7391;
        public static final double P = 0.48 * 128;
        public static final double I = 0;
        public static final double D = 0;
        public static final double G = 0.48;
        public static final double MotionMagicCruiseVelocity = 6.0;
        public static final double MotionMagicAcceleration = 8;
        public static final double Tolerance = 0.025;
        public static final double MaxHeight = ElevatorState.ALGAE_NET.getPosition();
    }

    public static final class IntakePivotConstants {
        public static final double PivotGearRatio = 52.0833;
        public static final double positionIncrement = 0;
        public static final double P = 84;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double G = 0.32;
        public static final double MotionMagicAcceleration = 30;
        public static final double MotionMagicCruiseVelocity = 2;
        public static final double MotionMagicJerk = 125;
        public static final double InitialPosition = 0.3186;
        public static final double EncoderOffset = 0;
    }

    public static final class IntakeRollersConstants {
        public static final double RollersGearRatio = 52.0833;
        public static final double stallCurrent = 0.0;
    }

    public static final class DrivetrainConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;

        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final double maxAngularRate = Math.PI * 3.5;
    }

    public static class ClimberConstants {
        // TODO: Find the values for these.
        public static final double DEPLOY_MOTION_MAGIC_CRUISE_VELOCITY = 0;
        public static final double DEPLOY_MOTION_MAGIC_ACCELERATION = 0;
        public static final double DEPLOY_MOTION_MAGIC_JERK = 0;
        public static final double CAGE_DETECECTION_CURRENT = 55;
        public static final double ClimberGearRatio = 0.0;
        public static final double EncoderOffset = 0.314;
    }

    public static final class HandConstants {
        public static final double intakeAlgaeStallCurrent = 50;
        public static final double hasAlgaeStallCurrent = 20;
    }

    public static final class CoralDetectorConstants {
        public static final double DETECTION_THRESHOLD = 0.065;
    }

    public static final class VisionConstants {
        public static final double xyStandardDeviation = 0.0;
        public static final double thetaStandardDeviation = 0.0;
    }
}
