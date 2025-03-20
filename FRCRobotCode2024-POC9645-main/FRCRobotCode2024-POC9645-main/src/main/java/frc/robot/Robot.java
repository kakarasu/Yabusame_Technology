package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivebase.manager.DriveManager;

import frc.robot.gyro.NavXGyro;
import frc.robot.upper.launcher.Launcher;
import frc.robot.util.input.GamePadManager;


public class Robot extends TimedRobot {

    private final EventLoop eventLoop = new EventLoop();

    private final XboxController rawGamepad = new XboxController(0);
    private final GamePadManager gamepad = new GamePadManager(rawGamepad);

    private DriveManager driveManager;
    private Launcher launcher;

    @Override
    public void robotInit() {
        driveManager = new DriveManager(gamepad);
        launcher = new Launcher(gamepad);
        NavXGyro.init();
        NavXGyro.initNTable();
    }

    @Override
    public void robotPeriodic() {
        eventLoop.poll();
        driveManager.poll(getPeriod());
        launcher.poll();
        NavXGyro.update();
    }

    @Override
    public void autonomousInit() {
        DriveManager.setLock(true);
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        DriveManager.setLock(false);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
