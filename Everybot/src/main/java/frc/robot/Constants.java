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
    public final static class DriveConstants {
        public final static int kRightFrontMotorPort = 0;
        public final static int kRightBackMotorPort = 0;
        public final static int kLeftFrontMotorPort = 0;
        public final static int kLeftBackMotorPort = 0;
    }
    public final static class ArmConstants {
        public final static int kArmMotor = 0;
    }
    public final static class IntakeConstants {
        public final static int kIntakeMotor = 0;
    }
    public final static class IOConstants {
        public final static int kDriverControllerPort = 0;
        public final static int kAButton = 1;
        public final static int kBButton = 2;
        public final static int kRBButton = 6;
        public final static int kLBButton = 5;
        public final static int kYButton = 4;
        public final static int kXButton = 3;
    }
}
