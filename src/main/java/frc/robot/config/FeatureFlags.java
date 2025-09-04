package frc.robot.config;

import java.util.function.BooleanSupplier;

import frc.robot.util.FeatureFlag;

public class FeatureFlags {
    public static final BooleanSupplier VISION_STALE_DATA_CHECK = FeatureFlag.of("Vision/StaleDataRejection", false);

    public static final BooleanSupplier MT_VISION_METHOD = FeatureFlag.of("Vision/MTVisionMethod", true);

    public static final BooleanSupplier CAMERA_POSITION_CALIBRATION = FeatureFlag.of("Vision/PositionCalibrationMode",false);

    public static final BooleanSupplier USE_ANY_REEF_TAG = FeatureFlag.of("Vision/UseAnyReefTag", true);

    public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);
}
