// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIRSTおよび他のWPILibの貢献者による著作権。
// オープンソースソフトウェアです。プロジェクトのルートディレクトリにあるWPILib BSDライセンスファイルの条件に従って、変更および共有できます。

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.AutonConstants;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  // スワーブドライブオブジェクト。
  private final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  // ロボットの最大速度（メートル毎秒）、加速度を制限するために使用されます。
  public double maximumSpeed = Units.feetToMeters(14.5);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  // 提供されたディレクトリで{@link SwerveDrive}を初期化します。
  // @param directory スワーブドライブ設定ファイルのディレクトリ。
  public SwerveSubsystem(File directory)
  {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    // 角度変換係数は360 / (ギア比 * エンコーダ解像度)です。
    // この場合、ギア比はホイール回転あたり12.8モーター回転です。
    // モーター回転あたりのエンコーダ解像度は1です。
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150/7);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    // モーター変換係数は(PI * ホイール直径（メートル）) / (ギア比 * エンコーダ解像度)です。
    // この場合、ホイール直径は4インチで、メートルに変換してメートル/秒を得る必要があります。
    // ギア比はホイール回転あたり6.75モーター回転です。
    // モーター回転あたりのエンコーダ解像度は1です。
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // 不要なオブジェクトの作成を避けるために、SwerveDriveを作成する前にテレメトリを設定します。
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // JSONファイルを介して変換係数を提供したくない場合の代替方法。
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
    // ヘディング補正は、角度を介してロボットを制御している間にのみ使用する必要があります。
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    // 実生活では見られない不一致を引き起こすため、シミュレーションではコサイン補正を無効にします。
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  // スワーブドライブを構築します。
  // @param driveCfg スワーブのためのSwerveDriveConfiguration。
  // @param controllerCfg スワーブコントローラー。
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  // PathPlannerのためのAutoBuilderを設定します。
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        // ロボットのポーズサプライヤー
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        // オドメトリをリセットするメソッド（自動運転が開始ポーズを持っている場合に呼び出されます）
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        // ChassisSpeedsサプライヤー。ロボット相対でなければなりません。
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        // ロボット相対のChassisSpeedsを与えられたロボットを駆動するメソッド
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         // HolonomicPathFollowerConfig、これはおそらくConstantsクラスに存在するべきです。
                                         AutonConstants.TRANSLATION_PID,
                                         // Translation PID constants
                                         // Translation PID定数
                                         AutonConstants.ANGLE_PID,
                                         // Rotation PID constants
                                         // Rotation PID定数
                                         4.5,
                                         // Max module speed, in m/s
                                         // 最大モジュール速度（m/s）
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         // ドライブベースの半径（メートル）。ロボットの中心から最も遠いモジュールまでの距離。
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
                                         // デフォルトのパス再計画設定。オプションについてはAPIを参照してください。
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          // パスがレッドアライアンス用にミラーリングされるタイミングを制御するブールサプライヤー
          // これにより、追従されるパスがフィールドの赤側に反転します。
          // 原点はブルーサイドに残ります。
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
        // 要件を設定するためのこのサブシステムへの参照
                                  );
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  // PhotonVisionによって返されたターゲットにロボットを向けます。
  // @param camera 通信するための{@link PhotonCamera}。
  // @return アライメントを実行する{@link Command}。
  public Command aimAtTarget(PhotonCamera camera)
  {
    return run(() -> {
      PhotonPipelineResult result = camera.getLatestResult();
      if (result.hasTargets())
      {
        drive(getTargetSpeeds(0,
                              0,
                              Rotation2d.fromDegrees(result.getBestTarget()
                                                           .getYaw()))); // Not sure if this will work, more math may be required.
        // これが機能するかどうかは不明です。さらに数学が必要かもしれません。
      }
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  // イベント付きのパスフォロワーを取得します。
  // @param pathName PathPlannerのパス名。
  // @return {@link AutoBuilder#followPath(PathPlannerPath)} パスコマンド。
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    // AutoBuilderを使用してパスフォローコマンドを作成します。これにより、イベントマーカーもトリガーされます。
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  // PathPlannerのパスファインディングを使用してフィールド上のポイントに移動します。
  // @param pose 移動先のターゲット{@link Pose2d}。
  // @return PathFindingコマンド
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    // パスファインディング中に使用する制約を作成します。
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // AutoBuilderが設定されているため、それを使用してパスファインディングコマンドを作成できます。
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        // 目標終端速度（メートル/秒）
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        // 回転遅延距離（メートル）。これは、回転を試みる前にロボットが移動する距離です。
                                     );
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  // 平行移動値とヘディングをセットポイントとして使用してロボットを駆動するコマンド。
  // @param translationX X方向の平行移動。スムーズな制御のために3乗されます。
  // @param translationY Y方向の平行移動。スムーズな制御のために3乗されます。
  // @param headingX ジョイスティックの角度を計算するためのヘディングX。
  // @param headingY ジョイスティックの角度を計算するためのヘディングY。
  // @return ドライブコマンド。
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    // 通常、この種の制御にはヘディング補正が必要です。
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      // スムーズな制御
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // スムーズな制御
      // Make the robot move
      // ロボットを動かす
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  // 平行移動値とヘディングをセットポイントとして使用してロボットを駆動するコマンド。
  // @param translationX X方向の平行移動。
  // @param translationY Y方向の平行移動。
  // @param rotation [-1, 1]の間の値としての回転をラジアンに変換します。
  // @return ドライブコマンド。
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    swerveDrive.setHeadingCorrection(false); // Normally you would want heading correction for this kind of control.
    // 通常、この種の制御にはヘディング補正が必要です。
    return run(() -> {
      // Make the robot move
      // ロボットを動かす
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  // SysIdを使用してロボットの駆動モーターを特性化するコマンド
  // @return SysIdドライブコマンド
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  // SysIdを使用してロボットの角度モーターを特性化するコマンド
  // @return SysId角度コマンド
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  // 平行移動値とヘディングを角速度として使用してロボットを駆動するコマンド。
  // @param translationX X方向の平行移動。スムーズな制御のために3乗されます。
  // @param translationY Y方向の平行移動。スムーズな制御のために3乗されます。
  // @param angularRotationX 設定するロボットの角速度。スムーズな制御のために3乗されます。
  // @return ドライブコマンド。
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      // ロボットを動かす
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  // ドライブベースを制御するための主要な方法。 {@link Translation2d}と回転速度を取り、それに応じてモジュールの状態を計算してコマンドを出します。
  // ホイール速度のオープンループまたはクローズドループの速度制御を使用できます。
  // また、フィールド相対モードとロボット相対モードがあり、平行移動ベクトルの使用方法に影響を与えます。
  // @param translation ロボットの指示された線速度（メートル毎秒）。ロボット相対モードでは、正のxは船首（前方）に向かい、正のyはポート（左）に向かいます。
  // フィールド相対モードでは、正のxはアライアンスウォール（フィールド北）から離れ、正のyはドライバーステーショングラスを通して見たときに左の壁に向かいます。
  // @param rotation ロボットの角速度（毎秒ラジアン）。CCWは正。フィールド/ロボットの相対性には影響されません。
  // @param fieldRelative ドライブモード。フィールド相対の場合はtrue、ロボット相対の場合はfalse。
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
    // オープンループは、ほとんどの時間で使用されるべきではないため、無効にされています。
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  // シャーシフィールド指向の速度を与えられたロボットを駆動します。
  // @param velocity フィールドに応じた速度。
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  // シャーシロボット指向の速度に従って駆動します。
  // @param velocity ロボット指向の{@link ChassisSpeeds}
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  // スワーブドライブのキネマティクスオブジェクトを取得します。
  // @return スワーブドライブの{@link SwerveDriveKinematics}。
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  // オドメトリを指定されたポーズにリセットします。このメソッドを呼び出すときにジャイロ角度とモジュール位置をリセットする必要はありません。
  // ただし、ジャイロ角度またはモジュール位置のいずれかがリセットされた場合、オドメトリが動作し続けるためにはこれを呼び出す必要があります。
  // @param initialHolonomicPose オドメトリを設定するポーズ
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  // オドメトリによって報告されたロボットの現在のポーズ（位置と回転）を取得します。
  // @return ロボットのポーズ
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  // クローズドループ速度制御でシャーシ速度を設定します。
  // @param chassisSpeeds 設定するシャーシ速度。
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  // トラジェクトリをフィールドに投稿します。
  // @param trajectory 投稿するトラジェクトリ。
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  // ジャイロ角度をゼロにリセットし、オドメトリを同じ位置にリセットしますが、0に向かって向けます。
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  // 駆動モーターをブレーキ/コーストモードに設定します。
  // @param brake モーターをブレーキモードに設定する場合はtrue、コーストモードに設定する場合はfalse。
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  // 基本的なドライブベースのスワーブポーズ推定器によって報告されたロボットの現在のヨー角を取得します。
  // これは生のジャイロ読み取りではなく、resetOdometry()の呼び出しから修正される場合があります。
  // @return ヨー角
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  // 2つのジョイスティックのコントローラー入力に基づいてシャーシ速度を取得します。1つは速度の方向用です。もう1つはロボットの角度用です。
  // @param xInput ロボットがX方向に移動するためのXジョイスティック入力。
  // @param yInput ロボットがY方向に移動するためのYジョイスティック入力。
  // @param headingX ロボットの角度を制御するXジョイスティック。
  // @param headingY ロボットの角度を制御するYジョイスティック。
  // @return スワーブドライブに送信できる{@link ChassisSpeeds}。
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  // 1つのジョイスティックと1つの角度のコントローラー入力に基づいてシャーシ速度を取得します。ロボットを90度のオフセットで制御します。
  // @param xInput ロボットがX方向に移動するためのXジョイスティック入力。
  // @param yInput ロボットがY方向に移動するためのYジョイスティック入力。
  // @param angle {@link Rotation2d}としての角度。
  // @return スワーブドライブに送信できる{@link ChassisSpeeds}。
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  // ロボットの現在のフィールド相対速度（x、y、オメガ）を取得します。
  // @return 現在のフィールド相対速度のChassisSpeedsオブジェクト
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  // ロボットの現在の速度（x、y、オメガ）を取得します。
  // @return 現在の速度の{@link ChassisSpeeds}オブジェクト
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  // スワーブドライブ内の{@link SwerveController}を取得します。
  // @return {@link SwerveDrive}からの{@link SwerveController}。
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  // {@link SwerveDriveConfiguration}オブジェクトを取得します。
  // @return 現在のドライブの{@link SwerveDriveConfiguration}。
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  // スワーブドライブをロックして移動を防ぎます。
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  // IMUによって報告されたロボットの現在のピッチ角を取得します。
  // @return {@link Rotation2d}角度としてのヘディング
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  // テスト目的で偽のビジョン読み取りを追加します。
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}
