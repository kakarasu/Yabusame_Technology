package frc.robot.util.rotation;

import edu.wpi.first.math.geometry.Rotation2d;

public class NormalizeRotation {

    public static double normalizeRotation(double rot){
        return rot % 1 + (rot < 0 ? 1 : 0);
    }

    public static Rotation2d normalizeRotation(Rotation2d rot) {
        double value = normalizeRadian(rot.getRadians());
        return new Rotation2d(value);
    }

    public static double normalizeDegree(double deg){
        return deg % 360 + (deg < 0 ? 360 : 0);
    }
    public static int normalizeDegree(int deg){
        return deg % 360 + (deg < 0 ? 360 : 0);
    }

    public static double normalizeRadian(double rad){
        return rad % (2 * Math.PI) + (rad < 0 ? 2*Math.PI : 0);
    }

    public static double normalizeRadianWithinHalf(double rad){
        return rad % (2 * Math.PI) + (rad < 0 ? Math.PI : -Math.PI);
    }

    public static Rotation2d normalizeRotationWithinPlusMinusHalf(Rotation2d rot){
        return Rotation2d.fromRadians(normalizeRadianWithinHalf(rot.getRadians()));
    }

    public static double normalizeRotationWithinPlusMinusHalf(double rot){
        return normalizeRotationWithinPlusMinusHalf(Rotation2d.fromRotations(rot)).getRotations();
    }



}
