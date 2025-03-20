package frc.robot.drivebase.swerveDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.drivebase.manager.DriveManager;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static frc.robot.drivebase.swerveDrive.SwerveDashboardWidgets.chassisAngle;
import static frc.robot.gyro.NavXGyro.gyroSensor;

public class SwerveDriveTrain {

    private final SwerveModule swerveFR = new SwerveModule(3, 4, 12, SwerveVariables.moduleLocations.FR.getValue(), "Front Right");
    private final SwerveModule swerveRR = new SwerveModule(5, 6, 13, SwerveVariables.moduleLocations.RR.getValue(), "Rear Right");
    private final SwerveModule swerveRL = new SwerveModule(7, 8, 14, SwerveVariables.moduleLocations.RL.getValue(), "Rear Left");
    private final SwerveModule swerveFL = new SwerveModule(1, 2, 11, SwerveVariables.moduleLocations.FL.getValue(), "Front Left");

    private final StructArrayPublisher<SwerveModuleState> driveTrainPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveDriveState", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveModuleState", SwerveModuleState.struct).publish();

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
            Rotation2d.fromDegrees(gyroSensor.getAngle()),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition(),
            }
        );

    public SwerveDriveTrain() {
        gyroSensor.reset();
        //gyroSensor.calibrate();
    }

    public void drive(double x, double y, double rotation, double periodSeconds) {
        SwerveModuleState[] state = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                /*SwerveVariables.fieldOriented*/ SwerveVariables.getFieldOriented()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(gyroSensor.getAngle()))
                    : new ChassisSpeeds(x, y, rotation), periodSeconds
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(state, SwerveVariables.getMaxSpeed());
        updatePublisher(state);

        //if((Math.abs(rotation) > 0.01&& Math.abs(x) > 0.01) || (Math.abs(rotation) > 0.01&& Math.abs(y) > 0.01)) {
            /*ChassisSpeeds speed = kinematics.toChassisSpeeds(state);

            state = kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    speed.plus(ChassisSpeeds.fromFieldRelativeSpeeds(x / 100, y / 100, 0, Rotation2d.fromDegrees(gyroSensor.getAngle()))),periodSeconds
                )
            );*/


        /*    swerveFL.setStateRotatingMoving(state[0],x,y,45,Rotation2d.fromDegrees(gyroSensor.getAngle()));
            swerveFR.setStateRotatingMoving(state[1],x,y,135,Rotation2d.fromDegrees(gyroSensor.getAngle()));
            swerveRL.setStateRotatingMoving(state[2],x,y,315,Rotation2d.fromDegrees(gyroSensor.getAngle()));
            swerveRR.setStateRotatingMoving(state[3],x,y,225,Rotation2d.fromDegrees(gyroSensor.getAngle()));
        }else{*/
        swerveFL.setState(state[0]);
        swerveFR.setState(state[1]);
        swerveRL.setState(state[2]);
        swerveRR.setState(state[3]);
        /*}*/

    }

    private void updatePublisher(SwerveModuleState[] state) {
        driveTrainPublisher.set(state);
        moduleStatePublisher.set(
            new SwerveModuleState[]{
                swerveFL.getState(),
                swerveFR.getState(),
                swerveRL.getState(),
                swerveRR.getState()
            }
        );
    }

    public void stop() {
        swerveFL.stop();
        swerveFR.stop();
        swerveRL.stop();
        swerveRR.stop();
    }

    public void updateWidget() {
        swerveFL.updateWidget();
        swerveFR.updateWidget();
        swerveRL.updateWidget();
        swerveRR.updateWidget();
        chassisAngle.setDouble(gyroSensor.getAngle());
    }

    public void reset() {
        /*DriveManager.setLock(true);
        try (ExecutorService threadPool = Executors.newFixedThreadPool(4)) {
            threadPool.submit(swerveFL::reset);
            threadPool.submit(swerveFR::reset);
            threadPool.submit(swerveRL::reset);
            threadPool.submit(swerveRR::reset);
        }finally {
            DriveManager.setLock(false);
        }*/
        swerveFL.reset();
        swerveFR.reset();
        swerveRL.reset();
        swerveRR.reset();
    }

    public void setX() {
        System.out.println("X");
        swerveFL.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
        swerveFR.setState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        swerveRL.setState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        swerveRR.setState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
    }


    public void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(gyroSensor.getAngle()),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition()
            });
    }

}
