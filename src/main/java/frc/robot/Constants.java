// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final int ROBOT_USING = 2023;

  public static class DrivetrainConstants{
    public static final int LEFT_TOP_MOTOR = 0;
    public static final int LEFT_BOTTOM_MOTOR = 1;
    public static final int RIGHT_TOP_MOTOR = 2;
    public static final int RIGHT_BOTTOM_MOTOR = 3;

    public static final int GEAR_SHIFTER_SOLENOID = 0;
  }

  public static class ElevatorConstants{
    public static final int ELEVATOR_MOTOR = 6;

    public static final int UPPER_ELEVATOR_LIMIT_SWITCH = 0;
    public static final int BOTTOM_ELEVATOR_LIMIT_SWITCH = 2;
  }

  public static class ClawConstants{
    public static final int LEFT_CLAW_ROLLER_MOTOR = 5;
    public static final int RIGHT_CLAW_ROLLER_MOTOR = 8;

    public static final int CLAW_SOLENOID = 1;

    public static final int CLAW_LIMIT_SWITCH = 8;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    
    public static final class OperationBinds{

      public static final int L_X_AXIS = 0;
      public static final int L_Y_AXIS = 1;
      public static final int L_TRIGGER = 2;
      public static final int R_TRIGGER = 3;
      public static final int R_X_AXIS = 4;
      public static final int R_Y_AXIS = 5;
    }
  }

  public static class ArmConstants{
    public static final int ARM_MOTOR = 7;
  }

}
