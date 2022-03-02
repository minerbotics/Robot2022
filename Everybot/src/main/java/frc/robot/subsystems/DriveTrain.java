// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  //Motor controllers
  private final CANSparkMax m_frontRMotor;
  private final CANSparkMax m_frontLMotor;
  private final CANSparkMax m_backRMotor;
  private final CANSparkMax m_backLMotor;
  //Motor controller groups
  private final MotorControllerGroup m_leftMotors;
  private final MotorControllerGroup m_rightMotors;
  //Encoders
  private final Encoder m_rightEncoder;
  private final Encoder m_leftEncoder;
  //Gyro
  private final Gyro m_gyro;

  private final DifferentialDrive m_drive;

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    super();
    //Motor controllers
    m_frontRMotor = new CANSparkMax(DriveConstants.kRightFrontMotorPort, MotorType.kBrushless);
    m_frontLMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorPort, MotorType.kBrushless);
    m_backRMotor = new CANSparkMax(DriveConstants.kRightBackMotorPort, MotorType.kBrushless);
    m_backLMotor = new CANSparkMax(DriveConstants.kLeftBackMotorPort, MotorType.kBrushless);
    //Motor controller groups
    m_leftMotors = new MotorControllerGroup(m_frontLMotor, m_backLMotor);
    m_rightMotors = new MotorControllerGroup(m_frontRMotor, m_backRMotor);
    //Drive
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    //Encoders
    m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1]);
    m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1]);
    //Gyro
    m_gyro = new AHRS(SerialPort.Port.kMXP);
    
    m_rightMotors.setInverted(true);
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void arcadeDrive(double forward, double rotation) {
    //rotation = rotation * 0.875;
    m_drive.arcadeDrive(forward, rotation, true);
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
