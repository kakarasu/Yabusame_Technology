package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {

  static void Main(){
    System.out.println("Robot Code Started.");
    RobotBase.startRobot(Robot::new);
  }

  public static void main(String... args) {
    Main();
  }
}
