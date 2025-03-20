package frc.robot.drivebase.simpleBackForwardSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.drivebase.swerveDrive.SwerveDriveTrain;
import frc.robot.drivebase.swerveDrive.SwerveVariables;

import static frc.robot.drivebase.swerveDrive.SwerveDashboardWidgets.chassisAngle;

public class SimpleBackForwardSwerveDriveTrain  {

    private final SimpleBackForwardSwerve FR = new SimpleBackForwardSwerve(3,"FR");
    private final SimpleBackForwardSwerve RR = new SimpleBackForwardSwerve(5,"RR");
    private final SimpleBackForwardSwerve RL = new SimpleBackForwardSwerve(7,"RL");
    private final SimpleBackForwardSwerve FL = new SimpleBackForwardSwerve(1,"FL");

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveDriveState", SwerveModuleState.struct).publish();

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            SwerveVariables.moduleLocations.FL.getValue(),
            SwerveVariables.moduleLocations.FR.getValue(),
            SwerveVariables.moduleLocations.RL.getValue(),
            SwerveVariables.moduleLocations.RR.getValue()
        );

    private final SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(0),
            new SwerveModulePosition[]{
                FL.getPosition(),
                FR.getPosition(),
                RL.getPosition(),
                RR.getPosition(),
            }
        );

    public void drive(double x, double periodSecond) {
        SwerveModuleState[] state = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                /*SwerveVariables.fieldOriented*/ SwerveVariables.getFieldOriented()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(x,0,0,Rotation2d.fromDegrees(0))
                    : new ChassisSpeeds(x,0,0),periodSecond
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(state,SwerveVariables.maxSpeed);

        publisher.set(state);


        FL.setState(state[0]);
        FR.setState(state[1]);
        RL.setState(state[2]);
        RR.setState(state[3]);
    }
    public void stop(){
        FL.stop();
        FR.stop();
        RL.stop();
        RR.stop();
    }

    public void updateWidget(){
        FL.updateWidget();
        FR.updateWidget();
        RL.updateWidget();
        RR.updateWidget();
        chassisAngle.setDouble(0);
    }
}
