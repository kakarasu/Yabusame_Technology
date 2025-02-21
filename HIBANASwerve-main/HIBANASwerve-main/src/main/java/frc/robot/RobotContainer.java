// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIRSTおよび他のWPILibの貢献者による著作権。
// オープンソースソフトウェアです。プロジェクトのルートディレクトリにあるWPILib BSDライセンスファイルの条件に従って、変更および共有できます。

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
// このクラスは、ロボットの大部分を宣言する場所です。コマンドベースは「宣言的」パラダイムであるため、実際にはロボットロジックは{@link Robot}の定期メソッド（スケジューラの呼び出しを除く）で処理されるべきではありません。
// 代わりに、ロボットの構造（サブシステム、コマンド、トリガーマッピングを含む）をここで宣言する必要があります。
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  // ロボットのサブシステムとコマンドはここで定義されます...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // 必要に応じてCommandPS4ControllerまたはCommandJoystickに置き換えます
  final CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  // ロボットのコンテナ。サブシステム、OIデバイス、およびコマンドを含みます。
  public RobotContainer()
  {
    // Configure the trigger bindings
    // トリガーバインディングを構成します
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // デッドバンドを適用し、制御を反転します。ジョイスティックは後ろ右が正であるのに対し、ロボットの制御は前左が正です。
    // 左スティックは平行移動を制御します。
    // 右スティックは希望の角度を制御します（角速度ではありません）。
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () ->  driverXbox.getHID().getRawButton(7) ? -driverXbox.getLeftY()*0.4: -driverXbox.getLeftY()*1,
        () ->  driverXbox.getHID().getRawButton(7) ? -driverXbox.getLeftX()*0.4: -driverXbox.getLeftX()*1,
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // デッドバンドを適用し、制御を反転します。ジョイスティックは後ろ右が正であるのに対し、ロボットの制御は前左が正です。
    // 左スティックは平行移動を制御します。
    // 右スティックはロボットの角速度を制御します。
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? (driverXbox.getHID().getRawButton(8) ? driveFieldOrientedDirectAngle : closedAbsoluteDriveAdv) : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  // このメソッドを使用して、トリガー->コマンドのマッピングを定義します。トリガーは、任意の述語を使用して{@link Trigger#Trigger(java.util.function.BooleanSupplier)}コンストラクターを介して、または
  // {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}のサブクラスの名前付きファクトリーを介して作成できます。
  // {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}コントローラーまたは{@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightジョイスティック}。
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // `exampleCondition`が`true`に変わると`ExampleCommand`をスケジュールします。

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // これを使用して、自律コマンドをメインの{@link Robot}クラスに渡します。
  // @return 自律運転で実行するコマンド
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // 自律運転で例のコマンドが実行されます
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}