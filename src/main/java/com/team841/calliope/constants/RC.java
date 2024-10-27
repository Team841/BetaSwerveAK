package com.team841.calliope.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

public class RC {

  public static Supplier<Boolean> isRedAlliance =
      () -> {
        var alliance = DriverStation.getAlliance();
          return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
      };

  public static boolean rumbleNeedsPing = false;

  public static final RunType robotType = RunType.COMP;

  public static Robot robot = Robot.CALLIOPE;

  public static boolean motorsAreWorking = true;

  public static class Controllers {
    public static final int soloStick = 3;
    public static final int duoStickDrive = 0;
    public static final int duoStickCoDrive = 1;

    public static final int SysIDCommandPort = 5; // for sysID
  }

  public static final class SC_CAN_ID {

    public static final int kIntake = 17;

    public static final int kIndexerTalon = 10;

    public static final int kHangerMotorLeft = 15;
    public static final int kHangerMotorRight = 3;

    public static final int kLeftArmJoint = 13;
    public static final int kRightArmJoint = 14;

    public static final int bottomShooter = 12;

    public static final int topShooter = 11;
  }

  public enum RunType{
    SIM, // Simulation
    DEV, // Developer-tuning mode
    COMP, // Comp code, real robot code
    REPLAY
  }

  public enum Robot{
      CALLIOPE,
        NIKE
  }
}
