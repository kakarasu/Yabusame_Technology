package frc.robot.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro {

    public static final AHRS gyroSensor = new AHRS(SPI.Port.kMXP);

    public static final Rotation2d offset = Rotation2d.fromDegrees(137);

    private static DoublePublisher publisher;

     public static void init(){
        gyroSensor.setAngleAdjustment(-offset.getDegrees());
    }

    public static void initNTable(){
        publisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("NavX Gyro Rotation (radians)").publish();
    }

    public static void update(){
        if(publisher != null){
            publisher.set(Rotation2d.fromDegrees(gyroSensor.getAngle()).getRadians());
        }
    }

}
