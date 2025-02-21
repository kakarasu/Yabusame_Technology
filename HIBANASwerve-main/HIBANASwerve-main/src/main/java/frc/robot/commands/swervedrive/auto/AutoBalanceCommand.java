package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * <a href="https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java">...</a>
 */
// 簡単なPIDコントローラーを使用した自動バランスコマンド。Team 3512によって作成されました。
public class AutoBalanceCommand extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   controller;

  public AutoBalanceCommand(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    controller = new PIDController(1.0, 0.0, 0.0);
    controller.setTolerance(1);
    controller.setSetpoint(0.0);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    // コマンドで使用される各サブシステムは、addRequirements()メソッドに渡す必要があります（Subsystemの可変引数を取ります）
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  // コマンドの初期サブルーチン。コマンドが最初にスケジュールされたときに一度だけ呼び出されます。
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  // コマンドのメインボディ。コマンドがスケジュールされている間、繰り返し呼び出されます。（つまり、{@link #isFinished()}がtrueを返すまで繰り返し呼び出されます。）
  @Override
  public void execute()
  {
    SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

    double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
                                           0.5);
    swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  // このコマンドが終了したかどうかを返します。一度コマンドが終了すると（このメソッドがtrueを返すことによって示されます）、スケジューラは{@link #end(boolean)}メソッドを呼び出します。
  // falseを返すと、コマンドは自動的に終了しなくなります。それでも手動でキャンセルされたり、他のコマンドによって中断されたりすることがあります。このコマンドを常にtrueを返すようにハードコーディングすると、コマンドは一度実行され、すぐに終了します。このような操作には{@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}を使用することをお勧めします。
  @Override
  public boolean isFinished()
  {
    return controller.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  // コマンドが終了したときに取るべきアクション。コマンドが正常に終了したとき（つまり、{@link #isFinished()}がtrueを返すとき）または中断/キャンセルされたときに呼び出されます。ここでは、コマンドで使用されていたモーターを停止するなどの未処理の部分をまとめることができます。
  // @param interrupted コマンドが中断/キャンセルされたかどうか
  @Override
  public void end(boolean interrupted)
  {
    swerveSubsystem.lock();
  }
}
