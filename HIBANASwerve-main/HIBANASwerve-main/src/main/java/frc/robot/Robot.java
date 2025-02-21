// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIRSTおよび他のWPILibの貢献者の著作権。
// オープンソースソフトウェアです。プロジェクトのルートディレクトリにあるWPILib BSDライセンスファイルの条件に従って、変更および共有できます。

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
// VMはこのクラスを自動的に実行し、TimedRobotドキュメントに記載されている各モードに対応する関数を呼び出すように構成されています。
// このプロジェクトを作成した後にクラス名またはパッケージを変更する場合は、プロジェクト内のbuild.gradleファイルも更新する必要があります。

public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  private VictorSPX intakeAngle=new VictorSPX(12);
  private CANSparkMax intake=new CANSparkMax(17, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rollerOutTop=new CANSparkMax(16, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rollerOutBottom=new CANSparkMax(18, CANSparkMax.MotorType.kBrushless);
  private GenericHID controller = new GenericHID(0);
  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  // ロボットが最初に起動したときに実行される関数で、初期化コードに使用する必要があります。
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // RobotContainerをインスタンス化します。これにより、すべてのボタンバインディングが実行され、
    // ダッシュボードに自律選択が配置されます。
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    // モーターブレーキを無効にするタイマーを作成します。無効になった後数秒後にこれにより、ロボットをすぐに停止させることができますが、
    // その後、さらに押すことができます
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   * この関数は、モードに関係なく20 msごとに呼び出されます。これは、無効、自律、テレオペレーション、テスト中に実行したい診断などのアイテムに使用します。
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
    * <p>これは、モード固有の周期関数の後、LiveWindowおよびSmartDashboard統合の更新前に実行されます。
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // スケジューラを実行します。これは、ボタンのポーリング、新しくスケジュールされたコマンドの追加、すでにスケジュールされたコマンドの実行、
    // 完了または中断されたコマンドの削除、およびサブシステムのperiodic()メソッドの実行に責任があります。
    // Command-basedフレームワーク内の何かが機能するためには、これをロボットの周期的なブロックから呼び出す必要があります。
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  // ロボットが無効モードに入るたびに一度だけ呼び出される関数です。
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  // この自律モードは、{@link RobotContainer}クラスによって選択された自律コマンドを実行します。
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // 自律コマンドをスケジュールします（例）
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   * この関数は、自律モード中に定期的に呼び出されます。
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
    // これにより、自律モードが実行されているときにテレオペが実行されるときに自律モードが停止します。
    // 自律モードを他のコマンドによって割り込まれるまで続行する場合は、この行を削除するかコメントアウトします。
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);

  }

  /**
   * This function is called periodically during operator control.
   */
  // この関数は、オペレーター制御中に定期的に呼び出されます。
  @Override
  public void teleopPeriodic()
  {
    //Amp -0.05 Bottom 0.4 Top
    //IntakeSource 0.4 Bottom, -0.4
    intake.set(controller.getRawButton(5) ? -0.2 : (controller.getRawButton(6) ? 0.1 : 0));
    if (controller.getPOV() == 0){
      rollerOutBottom.set(-0.05);
      rollerOutTop.set(0.4);
    }
    else if (controller.getPOV() == 90){
      rollerOutBottom.set(0.4);
      rollerOutTop.set(-0.4);
    }
    else{
      rollerOutBottom.set(0);
      rollerOutTop.set(0);
    }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    //テスト モードの開始時に実行中のすべてのコマンドをキャンセルします。
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
  // この関数は、テストモード中に定期的に呼び出されます。
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   * この関数は、ロボットが最初に起動したときに一度だけ呼び出されます。
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   * この関数は、シミュレーション中に定期的に呼び出されます。
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
