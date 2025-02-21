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
public class AbsoluteDrive extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private boolean initRotation = false;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
   *                          no deadband.  Positive is towards the left wall when looking through the driver station
   *                          glass.
   * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
   *                          with no deadband. Positive is away from the alliance wall.
   */
  // フィールド中心モードでスワーブロボットを駆動するために使用されます。vXとvYは翻訳入力を提供し、xはアライアンスウォールに向かって/離れており、yは左右です。headingHorzontalとheadingVerticalは、ロボットの角度が導き出されるデカルト座標です。これらは極角に変換され、ロボットが回転します。
  // @param swerve スワーブドライブベースのサブシステム。
  // @param vX x-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のXはアライアンスウォールから離れています。
  // @param vY y-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のYは、ドライバーステーショングラスを通して見たときに左の壁に向かっています。
  // @param headingHorizontal ロボットのヘディング角度の水平成分を提供するDoubleSupplier。ロボット座標系では、これはvYと同じ軸に沿っています。デッドバンドなしで-1から1の範囲である必要があります。正の値は、ドライバーステーショングラスを通して見たときに左の壁に向かっています。
  // @param headingVertical ロボットのヘディング角度の垂直成分を提供するDoubleSupplier。ロボット座標系では、これはvXと同じ軸に沿っています。デッドバンドなしで-1から1の範囲である必要があります。正の値はアライアンスウォールから離れています。
  public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
                       DoubleSupplier headingVertical)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // コマンドがスケジュールされている間、スケジューラが実行されるたびに呼び出されます。
  @Override
  public void execute()
  {

    // Get the desired chassis speeds based on a 2 joystick module.
    // 2つのジョイスティックモジュールに基づいて、希望のシャーシ速度を取得します。
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                         headingHorizontal.getAsDouble(),
                                                         headingVertical.getAsDouble());

    // Prevent Movement After Auto
    // 自動運転後の移動を防ぐ
    if (initRotation)
    {
      if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0)
      {
        // Get the curretHeading
        // 現在のヘディングを取得する
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        // 現在のヘディングを希望のヘディングに設定する
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      //Dont Init Rotation Again
      // 再度ヘディングをリセットしない
      initRotation = false;
    }

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
