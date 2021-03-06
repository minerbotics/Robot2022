// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
        public final static int kRightFrontMotorPort = 4;
        public final static int kRightBackMotorPort = 3;
        public final static int kLeftFrontMotorPort = 2;
        public final static int kLeftBackMotorPort = 1;
        public final static int[] kRightEncoderPorts = new int[] {0, 1};
        public final static int[] kLeftEncoderPorts = new int[] {2, 3};
        
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.0049191;
        public static final double kvVoltSecondsPerMeter = 0.22737;
        public static final double kaVoltSecondsSquaredPerMeter = 0.41717;
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.34521;
        public static final double kTrackwidthMeters = 0.504;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        

        public final static int kEncoderCPR = 1024;
        public final static double kWheelDiameterInches = 6;
        public final static double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    }
    public final static class ArmConstants {
        public final static int kArmMotor = 5;
    }
    public final static class IntakeConstants {
        public final static int kIntakeMotor = 6;
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
    public final static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
