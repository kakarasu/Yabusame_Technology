// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
// 例のサブシステムを使用する例のコマンド。
public class AbsoluteFieldDrive extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY, heading;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  // フィールド中心モードでスワーブロボットを駆動するために使用されます。vXとvYは翻訳入力を提供し、xはアライアンスウォールに向かって/離れており、yは左右です。headingHorzontalとheadingVerticalは、ロボットの角度が導き出されるデカルト座標です。これらは極角に変換され、ロボットが回転します。
  // @param swerve スワーブドライブベースのサブシステム。
  // @param vX x-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のXはアライアンスウォールから離れています。
  // @param vY y-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のYは、ドライバーステーショングラスを通して見たときに左の壁に向かっています。
  // @param heading ロボットのヘディング角度を提供するDoubleSupplier。
  public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier heading)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // コマンドがスケジュールされている間、スケジューラが実行されるたびに呼び出されます。
  @Override
  public void execute()
  {

    // Get the desired chassis speeds based on a 2 joystick module.
    // 2つのジョイスティックモジュールに基づいて、希望のシャーシ速度を取得します。
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                         new Rotation2d(heading.getAsDouble() * Math.PI));

    // Limit velocity to prevent tippy
    // 転倒を防ぐために速度を制限する
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    // ロボットを動かす
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

  }

  // Called once the command ends or is interrupted.
  // コマンドが終了するか中断されると一度だけ呼び出されます。
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  // コマンドが終了する必要があるときにtrueを返します。
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
