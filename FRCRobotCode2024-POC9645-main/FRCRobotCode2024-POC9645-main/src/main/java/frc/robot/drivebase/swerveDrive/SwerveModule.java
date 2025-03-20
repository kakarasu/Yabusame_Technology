package frc.robot.drivebase.swerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.util.rotation.NormalizeRotation;
import org.opencv.core.Mat;

import java.util.Map;

public class SwerveModule {

    private Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;

    private final CANcoder cancoder;

    private final PIDController drivePID = new PIDController(1, 0, 0);
    /*private final ProfiledPIDController rotatePID = new ProfiledPIDController(
        0.25,0,0,
        new TrapezoidProfile.Constraints(
            SwerveVariables.maxAngularSpeed, SwerveVariables.maxAngularSpeed * 2
        )
    );*/

    private final PIDController rotatePID = new PIDController(1,0,0);

    //private final PIDController rotatePID = new PIDController(1,0,0);

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.05, 0);
    private final SimpleMotorFeedforward rotateFeedforward = new SimpleMotorFeedforward(0.05, 0);

    private final String identifier;
    private GenericEntry driveMotorWidget;
    private GenericEntry rotateMotorWidget;

    private GenericEntry encoderWidget;
    private GenericEntry rotationWidget;

    private GenericEntry driveEncoderWidget;

    private GenericEntry desiredRotation;
    private Rotation2d lastAngle;

    public SwerveModule(int driveMotorChannel, int rotateMotorChannel, int CANCoderChannel, Translation2d position,
                        String identifierStr) {
        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
        rotateMotor = new CANSparkMax(rotateMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();

        //rotateMotor.setInverted(true);

        cancoder = new CANcoder(CANCoderChannel);

        identifier = identifierStr;

        driveEncoder.setVelocityConversionFactor(8.14);

        //rotatePID.enableContinuousInput(-Math.PI, Math.PI);

        initWidget();
    }

    private void initWidget() {
        driveMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Drive Motor " + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 0)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        rotateMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotate Motor " + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 1)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        encoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("CANCODER Value " + identifier, 0.0)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        rotationWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotation " + identifier, 0)
            .withPosition(2, 1)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("min", -180, "max", 180,"counterclockwise",true))
            .getEntry();

        driveEncoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("DRIVE ENCODER VALUE", 0)
            .getEntry();

        desiredRotation = Shuffleboard.getTab("Swerve " + identifier)
            .add("Desired Rot", 0)
            .getEntry();

    }

    public void updateWidget(double drive, double rotate) {
        encoderWidget.setDouble(cancoder.getAbsolutePosition().getValue());
        driveMotorWidget.setDouble(drive);
        rotateMotorWidget.setDouble(rotate);
        rotationWidget
            .setDouble(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()).getDegrees());
        driveEncoderWidget.setDouble(driveEncoder.getPosition());
    }

    public void updateWidget() {
        encoderWidget.setDouble(cancoder.getAbsolutePosition().getValue());
        driveMotorWidget.setDouble(driveMotor.getAppliedOutput());
        rotateMotorWidget.setDouble(rotateMotor.getAppliedOutput());
        rotationWidget
            .setDouble(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()).getDegrees());
        driveEncoderWidget.setDouble(driveEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity() / (8.14 * 60) * 2 * SwerveVariables.wheelRaduis * Math.PI;
        return new SwerveModuleState(
            driveMotor.getAppliedOutput(),
            Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition() / SwerveVariables.gearRatio
            * SwerveVariables.wheelRaduis * 2 * Math.PI/* add gear ratio calc */,
            Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()));
    }

    public void setState(SwerveModuleState state) {
        //Rotation2d currentRotation = new Rotation2d(NormalizeRotation.normalizeRotation(cancoder.getPosition().getValue()));
        Rotation2d currentRotation = NormalizeRotation.normalizeRotation(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()));

        SwerveModuleState newState = SwerveModuleState.optimize(state, currentRotation);
        //SwerveModuleState newState = SwerveModuleOptimizer.optimize(state,currentRotation);

        //SwerveModuleState newState = state;
        //SwerveModuleState newState = state;
        newState.speedMetersPerSecond *= newState.angle.minus(currentRotation).getCos();

        Rotation2d newAngle = (Math.abs(newState.speedMetersPerSecond) <= (SwerveVariables.getMaxSpeed()/100) )
            ? lastAngle:
            NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newState.angle);

        if(newAngle == null) {
            newAngle = newState.angle;
        }

        lastAngle = newAngle;

        newAngle = NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newAngle);

        double newspeedMetersPerSecond = driveEncoder.getVelocity() / (8.14 * 60) * 2 * SwerveVariables.wheelRaduis * Math.PI;

        //double driveOutput = drivePID.calculate(driveEncoder.getVelocity(), newState.speedMetersPerSecond);
        //double driveFeedForward = driveFeedforward.calculate(newState.speedMetersPerSecond);

        double rotateOutput = rotatePID.calculate(NormalizeRotation.normalizeRotationWithinPlusMinusHalf(currentRotation.getRotations()),NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newAngle.getRotations()));
        //double rotateOutput = NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newAngle.getRotations()) - NormalizeRotation.normalizeRotationWithinPlusMinusHalf(currentRotation.getRotations())

        double rotateFF = rotateFeedforward.calculate(rotatePID.getSetpoint());

        //double driveOutputValue = (driveOutput + driveFeedForward) / RobotController.getBatteryVoltage();
        //double driveOutputValue = driveOutput + driveFeedForward;
        double driveOutputValue = newState.speedMetersPerSecond;
        //double rotateOutputValue = (rotateOutput + rotateFeedForward) / RobotController.getBatteryVoltage();
        double rotateOutputValue = MathUtil.applyDeadband((rotateOutput/* + rotateFF*/) / 1,0);
        //updateWidget(driveOutputValue, rotateOutputValue);
        updateWidget(newState.speedMetersPerSecond, rotateOutputValue);
        desiredRotation.setDouble(newAngle.getRotations());

        if(Math.abs(newAngle.minus(currentRotation).getDegrees()) < 1.0){
            rotateOutputValue = 0;
        }

        //rotateMotor.set(newAngle.);
        driveMotor.set(driveOutputValue);
        rotateMotor.set(rotateOutputValue);
        //rotateMotor.set(newState.angle.getRadians() /10);

    }

    public void setStateRotatingMoving(SwerveModuleState state,double x,double y,int offset,Rotation2d theta){
        Rotation2d currentRotation = NormalizeRotation.normalizeRotation(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()));

        SwerveModuleState newState = SwerveModuleState.optimize(state, currentRotation);

        newState.speedMetersPerSecond *= newState.angle.minus(currentRotation).getCos();

        Rotation2d newAngle = (Math.abs(newState.speedMetersPerSecond) <= (SwerveVariables.getMaxSpeed()/100) )
            ? lastAngle:
            NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newState.angle);

        if(newAngle == null) {
            newAngle = newState.angle;
        }

        newAngle = NormalizeRotation.normalizeRotationWithinPlusMinusHalf(newAngle);

        double o = driveMotor.getAppliedOutput();

        double nuke = theta.plus(Rotation2d.fromDegrees(offset)).getRadians();

        double unkox = x+o*Math.cos(nuke);
        double unkoy = y+o* Math.sin(nuke);

        double rotateOutput = Math.atan(unkoy/unkox) / (2*Math.PI);
        double rotateOutputValue = rotatePID.calculate(currentRotation.getRotations(),rotateOutput);

        driveMotor.set(Math.sqrt(Math.pow(unkox,2)+Math.pow(unkoy,2)));
        rotateMotor.set(rotateOutputValue);

        updateWidget(newState.speedMetersPerSecond, rotateOutput);

    }

    public void stop() {

        driveMotorWidget.setDouble(0);
        rotateMotorWidget.setDouble(0);

        if (driveMotor != null)
            driveMotor.set(0);
        if (rotateMotor != null)
            rotateMotor.set(0);
    }

    public void reset(){
        /*System.out.println("called");
        boolean completed = false;
        while (!completed){
            Rotation2d currentRotation = NormalizeRotation.normalizeRotation(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue()));
            double rotateOutput = rotatePID.calculate(NormalizeRotation.normalizeRotationWithinPlusMinusHalf(currentRotation.getRotations()),0);
            rotateMotor.set(rotateOutput);
            if(Math.abs(currentRotation.getRotations()) < Rotation2d.fromDegrees(1).getRotations())
                completed = true;
        }*/
    }
}
