package frc.robot.drivebase.swerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.rotation.NormalizeRotation;

public class SwerveModuleOptimizer {

    public static SwerveModuleState optimize(SwerveModuleState state, Rotation2d rot) {
        Rotation2d stateAngle = NormalizeRotation.normalizeRotation(state.angle);
        Rotation2d currentRot = NormalizeRotation.normalizeRotation(rot);
        return Math.abs(stateAngle.minus(currentRot).getRotations()) > 0.25
            ? new SwerveModuleState(-state.speedMetersPerSecond, state.angle.rotateBy(new Rotation2d(0.5)))
            : state;
    }
}
