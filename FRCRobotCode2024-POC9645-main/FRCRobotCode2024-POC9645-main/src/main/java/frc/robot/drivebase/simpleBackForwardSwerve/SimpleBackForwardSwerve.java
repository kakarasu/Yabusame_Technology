package frc.robot.drivebase.simpleBackForwardSwerve;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.drivebase.swerveDrive.SwerveVariables;

public class SimpleBackForwardSwerve {

    private final CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;

    //private final PIDController drivePID = new PIDController(1, 0, 0);

    private GenericEntry driveMotorWidget;

    public  SimpleBackForwardSwerve(int drivech,String id){
        driveMotor = new CANSparkMax(drivech, CANSparkLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();

        if (drivech == 1 || drivech == 5) {
            driveMotor.setInverted(true);
        }
        initWidget(id);
    }

    public void initWidget(String id){
        driveMotorWidget = Shuffleboard.getTab("Swerve " + id)
            .add("Drive Motor " + id , 0.0)
            .withSize(2, 1)
            .withPosition(0, 0)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();
    }

    public void updateWidget(double drive) {
        driveMotorWidget.setDouble(drive);
    }

    public void updateWidget() {
        driveMotorWidget.setDouble(driveMotor.getAppliedOutput());
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition() / SwerveVariables.gearRatio
            * SwerveVariables.wheelRaduis * 2 * Math.PI/* add gear ratio calc */,
            new Rotation2d(0));
    }

    public void setState(SwerveModuleState state) {
        Rotation2d currentRotation = new Rotation2d(0);

        SwerveModuleState newState = SwerveModuleState.optimize(state, currentRotation);

        state.speedMetersPerSecond *= state.angle.minus(currentRotation).getCos();

        double driveOutput = state.speedMetersPerSecond;

        updateWidget(driveOutput > 0 ? 0.15 : -0.15);
        System.out.println(driveOutput > 0 ? 0.15 : -0.15);
        driveMotor.set(driveOutput > 0 ? 0.15 : -0.15);
        // rotateMotor.set(rotateOutputValue);

    }

    public void stop() {

        driveMotorWidget.setDouble(0);

        if (driveMotor != null)
            driveMotor.set(0);
    }


}
