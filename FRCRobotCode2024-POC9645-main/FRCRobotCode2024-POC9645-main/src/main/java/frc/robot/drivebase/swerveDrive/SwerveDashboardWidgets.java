package frc.robot.drivebase.swerveDrive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Map;

public class SwerveDashboardWidgets {
    public static GenericEntry fieldOrientedWidget;

    public static GenericEntry maxSpeedWidget;

    public static GenericEntry maxAngularSpeedWidget;

    public static ShuffleboardLayout inputContainer;
    public static ShuffleboardLayout outputContainer;

    public static GenericEntry inputX;
    public static GenericEntry inputY;
    public static GenericEntry inputRot;

    public static GenericEntry outputX;
    public static GenericEntry outputY;
    public static GenericEntry outputRot;

    public static GenericEntry chassisAngle;


    public static void initShuffleboardLayout() {
        fieldOrientedWidget = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Field Oriented Drive", SwerveVariables.fieldOriented)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0,0)
            .getEntry();

        maxSpeedWidget = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Max Speed", SwerveVariables.maxSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1.0))
            .withPosition(1,0)
            .getEntry();

        maxAngularSpeedWidget = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Max Angular Speed(radian)",SwerveVariables.maxAngularSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min",0,"max",Math.PI))
            .withPosition(1,1)
            .getEntry();

        inputContainer = Shuffleboard.getTab("Swerve DriveTrain")
            .getLayout("DriveTrain Input", BuiltInLayouts.kList)
            .withSize(1, 2)
            .withPosition(3,0);

        inputX = inputContainer.add("Input X", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
        inputY = inputContainer.add("Input Y", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
        inputRot = inputContainer.add("Input Rot", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        outputContainer = Shuffleboard.getTab("Swerve DriveTrain")
            .getLayout("DriveTrain Output", BuiltInLayouts.kList)
            .withSize(1, 2)
            .withPosition(4,0);

        outputX = outputContainer.add("Output X", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
        outputY = outputContainer.add("Output Y", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
        outputRot = outputContainer.add("Output Rot", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        chassisAngle = Shuffleboard.getTab("Swerve DriveTrain")
            .add("NaxX Gyro", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(5,0)
            .getEntry();

    }
}
