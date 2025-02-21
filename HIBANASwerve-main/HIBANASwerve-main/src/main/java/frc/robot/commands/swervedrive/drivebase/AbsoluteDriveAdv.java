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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
// どの方向を向くかを決定する4つのボタンを備えた、より高度なスワーブ制御システム
public class AbsoluteDriveAdv extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  headingAdjust;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
  private       boolean         resetHeading = false;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
   * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param headingAdjust DoubleSupplier that supplies the component of the robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband already accounted for.
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   */
  // フィールド中心モードでスワーブロボットを駆動するために使用されます。vXとvYは翻訳入力を提供し、xはアライアンスウォールに向かって/離れており、yは左右です。Heading Adjustは、定数で乗算された後に現在のヘディングを変更します。lookブール値は、ロボットを特定の方向に向けるためのショートカットです。https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172のアイデアに基づいています。
  // @param swerve スワーブドライブベースのサブシステム。
  // @param vX x-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のXはアライアンスウォールから離れています。
  // @param vY y-翻訳ジョイスティック入力を提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。正のYは、ドライバーステーショングラスを通して見たときに左の壁に向かっています。
  // @param headingAdjust ロボットのヘディング角度の調整するコンポーネントを提供するDoubleSupplier。デッドバンドがすでに考慮されている範囲-1から1の範囲である必要があります。
  // @param lookAway ドライバーが向いているのと同じ方向にロボットを対戦アライアンスの壁に向ける
  // @param lookTowards ドライバーに向かってロボットを向ける
  // @param lookLeft ロボットを左に向ける
  // @param lookRight ロボットを右に向ける
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    resetHeading = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // コマンドがスケジュールされている間、スケジューラが実行されるたびに呼び出されます。
  @Override
  public void execute()
  {
    double headingX = 0;
    double headingY = 0;

    // These are written to allow combinations for 45 angles
    // これらは45度の角度の組み合わせを許可するために書かれています
    // Face Away from Drivers
    // ドライバーから離れる方向を向く
    if (lookAway.getAsBoolean())
    {
      headingY = -1;
    }
    // Face Right
    // 右を向く
    if (lookRight.getAsBoolean())
    {
      headingX = 1;
    }
    // Face Left
    // 左を向く
    if (lookLeft.getAsBoolean())
    {
      headingX = -1;
    }
    // Face Towards the Drivers
    // ドライバーに向かって顔を向ける
    if (lookTowards.getAsBoolean())
    {
      headingY = 1;
    }

    // Prevent Movement After Auto
    // 自動運転後の移動を防ぐ
    if (resetHeading)
    {
      if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
      {
        // Get the curret Heading
        // 現在のヘディングを取得する
        Rotation2d currentHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        // 現在のヘディングを希望のヘディングに設定する
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      //Dont reset Heading Again
      // 再度ヘディングをリセットしない
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

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
    if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
    {
      resetHeading = true;
      swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
    } else
    {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
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
