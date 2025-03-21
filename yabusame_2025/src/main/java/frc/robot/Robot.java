// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;

// import com.revrobotics.SparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  //private        Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  private SparkMax right_arm=new SparkMax(18, SparkMax.MotorType.kBrushless);
  private SparkMax left_arm=new SparkMax(17, SparkMax.MotorType.kBrushless);
  //private SparkMax intake=new SparkMax(17, SparkMax.MotorType.kBrushless);
  //private SparkMax rollerOutTop=new SparkMax(16, SparkMax.MotorType.kBrushless);
  //private SparkMax rollerOutBottom=new SparkMax(18, SparkMax.MotorType.kBrushless);
  private GenericHID controller = new GenericHID(0);
  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    System.out.println("1");
    try{
      m_robotContainer = new RobotContainer();
      System.out.println("999");
    }catch(Exception e){
      System.out.println("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW in robotInit in RobotContainer");
      System.out.println("");
      e.printStackTrace();
    }
    System.out.println("8");
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
   // immediately when disabled, but then also let it be pushed more 
   try{
    disabledTimer = new Timer();
   }catch(Exception e){
    System.out.println("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW in robotInit in Timer");
    System.out.println("");
    e.printStackTrace();
  }
    System.out.println("9");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    if(m_robotContainer != null)
    {
      m_robotContainer.setMotorBrake(true);
    }
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      if(m_robotContainer != null){
        m_robotContainer.setMotorBrake(false);
      }
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    if(m_robotContainer != null)
    {
      m_robotContainer.setDriveMode();
      m_robotContainer.setMotorBrake(true);
    }
    else{
      System.out.println("m_robotContainer is null");
    }
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //// schedule the autonomous command (example)
    //if (m_autonomousCommand != null)
    //{
    //  m_autonomousCommand.schedule();
    //}
    System.out.println("Autonomous Init");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //if (m_autonomousCommand != null)
    //{
    //  m_autonomousCommand.cancel();
    //}
    if(m_robotContainer != null)
    {
      m_robotContainer.setDriveMode();
      m_robotContainer.setMotorBrake(true);
    }
    else{
      System.out.println("m_robotContainer is null");
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
//############################################################################### 編集箇所 (手動モード アーム) ###############################################################################
  //  //コントローラーのYボタンを押すとアームが上がる
    if(controller.getRawButton(4)){
      right_arm.set(0.5);
      left_arm.set(-0.5);
    }
    //コントローラーのAタンを押すとアームが下がる
    else if(controller.getRawButton(1)){
      right_arm.set(-0.5);
      left_arm.set(0.5);
    }
    //それ以外の場合、停止
    else{
      right_arm.set(0.0);
      left_arm.set(0.0);
    }
//############################################################################### 編集箇所ここまで (編集者:池田) ###############################################################################
    //Amp -0.05 Bottom 0.4 Top
    //IntakeSource 0.4 Bottom, -0.4
    //intake.set(controller.getRawButton(5) ? -0.2 : (controller.getRawButton(6) ? 0.1 : 0));
    //if (controller.getPOV() == 0){
    //  rollerOutBottom.set(-0.05);
    //  rollerOutTop.set(0.4);
    //}
    //else if (controller.getPOV() == 90){
    //  rollerOutBottom.set(0.4);
    //  rollerOutTop.set(-0.4);
    //}
    //else{
    //  rollerOutBottom.set(0);
    //  rollerOutTop.set(0);
    //}
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
