package frc.robot.util;

public class RotationUtil {

    /**
     * Converts a rotation (in degrees) to an unsigned value from 0 to 360 degrees.
     *
     * @return unsigned rotation
     */
    public static double toUnsignedDegrees(double degrees) {
        double x = degrees % 360;
        return x < 0
               ? x + 360
               : x;
    }

    /**
     * Converts a rotation (in degrees) to a signed value from -180 to 180 degrees.
     *
     * @return signed rotation
     */
    public static double toSignedDegrees(double degrees) {
        double x = toUnsignedDegrees(degrees);
        return x > 180
               ? x - 360
               : x;
    }
}