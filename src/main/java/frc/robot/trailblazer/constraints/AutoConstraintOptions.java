package frc.robot.trailblazer.constraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * @param maxLinearVelocity      Max linear velocity allowed in meters per second. Set to 0 to disable.
 * @param maxAngularVelocity     Max angular velocity allowed in radians per second. Set to 0 to disable.
 * @param maxLinearAcceleration  Max linear acceleration allowed in meters per second squared. Set to 0 to disable.
 * @param maxAngularAcceleration Max angular acceleration allowed in radians per second squared. Set to 0 to disable.
 */
public record AutoConstraintOptions(
        double maxLinearVelocity,
        double maxAngularVelocity,
        double maxLinearAcceleration,
        double maxAngularAcceleration) {

    public AutoConstraintOptions withMaxLinearVelocity(double maxLinearVelocity) {
        return new AutoConstraintOptions(
                maxLinearVelocity, maxAngularVelocity(), maxLinearAcceleration(), maxAngularAcceleration());
    }

    public AutoConstraintOptions withMaxAngularVelocity(double maxAngularVelocity) {
        return new AutoConstraintOptions(
                maxLinearVelocity(), maxAngularVelocity, maxLinearAcceleration(), maxAngularAcceleration());
    }

    public AutoConstraintOptions withMaxLinearAcceleration(double maxLinearAcceleration) {
        return new AutoConstraintOptions(
                maxLinearVelocity(), maxAngularVelocity(), maxLinearAcceleration, maxAngularAcceleration());
    }

    public AutoConstraintOptions withMaxAngularAcceleration(double maxAngularAcceleration) {
        return new AutoConstraintOptions(
                maxLinearVelocity(), maxAngularVelocity(), maxLinearAcceleration(), maxAngularAcceleration);
    }

    public TrapezoidProfile.Constraints getLinearConstraints() {
        return new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration);
    }

    public TrapezoidProfile.Constraints getAngularConstraints() {
        return new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);
    }
}
