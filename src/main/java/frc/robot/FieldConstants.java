package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static class StagingPositions {
        private static final double fieldWidth = 8.052;
        // Measured from the center of the ice cream
        public static final double separation = Units.inchesToMeters(72.0);
        public static final Translation2d[] iceCreams = new Translation2d[3];

        static {
            for (int i = 0; i < 3; i++) {
                iceCreams[i] =
                        new Translation2d(
                                Units.inchesToMeters(48),
                                fieldWidth / 2.0 - separation + separation * i);
            }
        }
    }

    public enum PipeScoringLevel {
        L2,
        L3,
        L4
    }
}
