package frc.robot.drivebase.swerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

public class SwerveVariables {

    // DO NOT USE THESE VALUES TO CALCULATE SOMETHING.
    // PLEASE GET VALUES FROM GETTER BELOW OR DASHBOARD WIDGETS INSTEAD

    // 5.08 cm, 2 in
    public static final double wheelRaduis = Units.Meters.convertFrom(5.08, Units.Centimeter);

    // spark max encoder with brushless-motor reports 42 counts per rotation
    public static final int brushlessMotorResolution = 42;

    //45 degrees
    public static double maxAngularSpeed = Math.PI / 4;

    //20% of Max motor output
    public static final double maxSpeed = 0.2;

    public static final boolean fieldOriented = false;

    // Gear ratio motor rev : wheel rev = 8.14:1
    public static double gearRatio = 8.14;

    public static boolean getFieldOriented() {
        return SwerveDashboardWidgets.fieldOrientedWidget.getBoolean(fieldOriented);
    }

    public static double getMaxSpeed() {
        return SwerveDashboardWidgets.maxSpeedWidget.getDouble(maxSpeed);
    }

    public static double getMaxAngularSpeed(){
        return SwerveDashboardWidgets.maxAngularSpeedWidget.getDouble(Math.PI/4);
    }

    public enum moduleLocations {
        FL(0.5, 0.5),
        FR(0.5, -0.5),
        RL(-0.5, 0.5),
        RR(-0.5, -0.5);

        private final Translation2d value;

        moduleLocations(double x, double y) {
            this.value = new Translation2d(x, y);
        }

        public Translation2d getValue() {
            return value;
        }
    }

}
