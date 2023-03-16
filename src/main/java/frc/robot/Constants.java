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
  public static class CAN_IDs {
    public static final int driveTrain_Left1 = 11;
    public static final int driveTrain_Left2 = 12;
    public static final int driveTrain_Right1 = 13;
    public static final int driveTrain_Right2 = 14;
    public static final int intakeWheels = 21;
    public static final int intakeArm = 22;
    public static final int elevatorLeft = 23;
    public static final int elevatorRight = 24;
    public static final int slider = 25;
  }
  public static class DriveTrainConstants {
    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kCurrentLimit = 55;

    public static final double kTurningScale = 0.5;

    public static final double kWheelDiameter = 6.0; // in inches
    public static final double _gearRaio = (11/52) * (30/68); // == 0.0933 = 1/10.715
    // kPositionFactor converts # of motor rotations to distance travelled
    public static final double kPositionFactor = _gearRaio * Math.PI * kWheelDiameter;
    // kVelocityFactor converts # of motor RPM to inches/sec
    public static final double kVelocityFactor = kPositionFactor / 60;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort1 = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmManualDeadband = 0.05;
    public static final double kArmManualScale = 0.5;
  }
}
