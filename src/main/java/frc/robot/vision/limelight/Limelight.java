package frc.robot.vision.limelight;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.ReusableOptional;
import frc.robot.vision.results.OptionalTagResult;

import java.util.OptionalDouble;

import com.pathplanner.lib.config.RobotConfig;

public class Limelight extends StateMachine<LimelightStates> {
    private static final int[] VALID_APRILTAGS = new int[] { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    private static final double IS_OFFLINE_TIMEOUT = 3;
    private static final double USE_MT1_DISTANCE_THRESHOLD = Units.inchesToMeters(40.0);
    private final String limelightTableName;
    private final String name;
    private final boolean mt1Compatible;

    private final Timer limelightTimer = new Timer();
    private double limelightHeartbeat = -1;

    private double lastTimestamp = 0.0;

    private OptionalTagResult lastGoodTagResult = new OptionalTagResult();
    private OptionalTagResult tagResult = new OptionalTagResult();

    private double angularVelocity = 0.0;

    private final int[] closestScoringReefTag = { 0 };

    public Limelight(
            String name,
            LimelightStates initialState,
            boolean mt1Compatible) {
        // not just
        // singleton
        super(initialState);
        limelightTableName = "limelight-" + name;
        this.name = name;
        limelightTimer.start();
        this.mt1Compatible = mt1Compatible;
    }

    public void sendImuData(
            double robotHeading,
            double angularVelocity,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        LimelightHelpers.SetRobotOrientation(
                limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
        this.angularVelocity = angularVelocity;
    }

    public void setState(LimelightStates state) {
        setStateFromRequest(state);
    }

    public OptionalTagResult getTagResult() {
        if (getState() != LimelightStates.TAGS && getState() != LimelightStates.CLOSEST_REEF_TAG) {
            return tagResult.empty();
        }

        var mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);
        if (mT2Estimate == null) {
            return tagResult.empty();
        }
        if (Math.abs(angularVelocity) > 360) {
            return tagResult.empty();
        }
        if (mT2Estimate.tagCount == 0) {
            DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

            return tagResult.empty();
        }
        if (mT2Estimate.rawFiducials.length == 1) {
            double ambiguity = mT2Estimate.rawFiducials[0].ambiguity;
            if (ambiguity >= 0.7) {
                return tagResult.empty();
            }
        }
        DogLog.log("Vision/" + name + "/Tags/MT2Timestamp", mT2Estimate.timestampSeconds);
        if (FeatureFlags.VISION_STALE_DATA_CHECK.getAsBoolean()) {
            var newTimestamp = mT2Estimate.timestampSeconds;
            if (newTimestamp == lastTimestamp) {
                DogLog.log("Vision/" + name + "/Tags/MT2Timestamp", 0.0);

                DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
                return tagResult.empty();
            }

            lastTimestamp = newTimestamp;
        }

        var mt2Pose = mT2Estimate.pose;
        // This prevents pose estimator from having crazy poses if the Limelight loses
        // power
        if (mt2Pose.getX() == 0.0 && mt2Pose.getY() == 0.0) {
            DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

            return tagResult.empty();
        }
        var devs = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
        if (mt1Compatible && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
            var distance = mT2Estimate.avgTagDist;
            DogLog.log("Vision/" + name + "/Tags/DistanceFromTag", Units.metersToInches(distance));

            var xyDev = 0.01 * Math.pow(distance, 1.2);
            var thetaDev = 0.03 * Math.pow(distance, 1.2);

            devs = VecBuilder.fill(xyDev, xyDev, thetaDev);

            if (distance <= USE_MT1_DISTANCE_THRESHOLD) {
                DogLog.timestamp("Vision/" + name + "/Tags/UsingMT1Rotation");
                var mT1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
                if (mT1Result != null
                        && mT1Result.tagCount != 0
                        && mT1Result.pose.getRotation().getDegrees() != 0.0) {
                    mt2Pose = new Pose2d(mT2Estimate.pose.getTranslation(), mT1Result.pose.getRotation());
                }
            }
        }

        DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", mt2Pose);
        return tagResult.update(mt2Pose, mT2Estimate.timestampSeconds, devs);
    }

    public void setClosestScoringReefTag(int tagID) {
        closestScoringReefTag[0] = tagID;
    }

    @Override
    protected void collectInputs() {
        tagResult = getTagResult();
        if (tagResult.isPresent()) {
            lastGoodTagResult = tagResult;
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Vision/" + name + "/State", getState());

        var lastTagTimestamp = lastGoodTagResult.isPresent()
                ? lastGoodTagResult.orElseThrow().timestamp()
                : Double.MIN_VALUE;

        if (Timer.getTimestamp() - lastTagTimestamp > 30) {
            DogLog.logFault(
                    limelightTableName + " has not seen a tag in the last 30 seconds", AlertType.kWarning);
        }

        LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
        switch (getState()) {
            case TAGS -> {
                LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
                updateHealth(tagResult);
            }
            case CLOSEST_REEF_TAG -> {
                LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, closestScoringReefTag);
                updateHealth(tagResult);
            }
        }

        // TODO: Remove once Limelights are upgraded
        LimelightHelpers.SetIMUMode(limelightTableName, 0);
    }

    private void updateHealth(ReusableOptional<?> result) {
        var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
        DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
        if (limelightHeartbeat != newHeartbeat) {
            limelightTimer.restart();
        }
        limelightHeartbeat = newHeartbeat;
    }

    // public void setBlinkEnabled(boolean enabled) {
    //     if (enabled) {
    //         LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    //     } else {
    //         LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    //     }

    public void logCameraPositionCalibrationValues() {
        var cameraPoseTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(limelightTableName);
        var robotPoseTargetSpace = new Pose3d() ; //TODO: Find Robot pose relative to to calibration
        var cameraRobotRelativePose = getRobotRelativeCameraPosition(robotPoseTargetSpace, cameraPoseTargetSpace);
        DogLog.log("CameraPositionCalibration/" + name + "/LL Right", cameraRobotRelativePose.getX());
        DogLog.log("CameraPositionCalibration/" + name + "/LL Up", cameraRobotRelativePose.getY());
        DogLog.log("CameraPositionCalibration/" + name + "/LL Forward", cameraRobotRelativePose.getZ());
        DogLog.log(
                "CameraPositionCalibration/" + name + "/LL Roll",
                Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getX()));
        DogLog.log(
                "CameraPositionCalibration/" + name + "/LL Pitch",
                Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getY()));
        DogLog.log(
                "CameraPositionCalibration/" + name + "/LL Yaw",
                Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getZ()));
    }

    private static Pose3d getRobotRelativeCameraPosition(
            Pose3d robotPoseTargetSpace, Pose3d seenCameraPoseTargetSpace) {
        // Positive X = Right
        var cameraLeftRight = seenCameraPoseTargetSpace.getX();
        // Positive Y = Down, so flipped for common sense
        var cameraUpDown = -1 * seenCameraPoseTargetSpace.getY();
        // Positive Z = Forward
        var cameraForwardBackward = seenCameraPoseTargetSpace.getZ();
        // Pitch rotates around left right axis (x according to LL coordinate systems)
        var cameraPitch = seenCameraPoseTargetSpace.getRotation().getX();
        // Roll rotates around forward backward axis (Z according to LL coordinate
        // systems)
        var cameraRoll = seenCameraPoseTargetSpace.getRotation().getZ();
        // Yaw rotates around up down axis (y according to LL coordinate systems)
        var cameraYaw = -1 * seenCameraPoseTargetSpace.getRotation().getY();

        var robotLeftRight = robotPoseTargetSpace.getX();
        var robotUpDown = robotPoseTargetSpace.getY();
        var robotForwardBackward = robotPoseTargetSpace.getZ();
        var robotPitch = robotPoseTargetSpace.getRotation().getY();
        var robotRoll = robotPoseTargetSpace.getRotation().getX();
        var robotYaw = robotPoseTargetSpace.getRotation().getZ();

        var right = cameraLeftRight - robotLeftRight;
        var up = cameraUpDown - robotUpDown;
        var forward = cameraForwardBackward - robotForwardBackward;
        var roll = cameraRoll - robotRoll;
        var pitch = cameraPitch - robotPitch;
        var yaw = cameraYaw - robotYaw;

        return new Pose3d(right, up, forward, new Rotation3d(roll, pitch, yaw));
    }
}