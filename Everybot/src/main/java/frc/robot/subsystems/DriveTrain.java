// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forward, double rotation) {
    //rotation = rotation * 0.875;
    m_drive.arcadeDrive(forward, rotation, true);
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
}
