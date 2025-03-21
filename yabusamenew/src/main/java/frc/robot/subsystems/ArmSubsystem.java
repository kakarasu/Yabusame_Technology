// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig sparkMaxConfig;

    /**
     * Creates a new ExampleSubsystem.
     */
    public ArmSubsystem() {
        armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        closedLoopController = armMotor.getClosedLoopController();
        encoder = armMotor.getEncoder();
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.encoder
                .positionConversionFactor(1);
        sparkMaxConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .p(0.03)
                .i(0)
                .d(0)
                .maxOutput(0.2)
                .minOutput(-0.2)
                .outputRange(-1, 1);
        armMotor.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Current Position", 0);

    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public void Up() {
        closedLoopController.setReference(-12, SparkBase.ControlType.kPosition);
        SmartDashboard.putNumber("Target Position", -12);
    }

    public void Down() {
        closedLoopController.setReference(0, SparkBase.ControlType.kPosition);
        SmartDashboard.putNumber("Target Position", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Position", encoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
