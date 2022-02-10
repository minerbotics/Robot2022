// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax m_frontRMotor;
  private final CANSparkMax m_frontLMotor;
  private final CANSparkMax m_backRMotor;
  private final CANSparkMax m_backLMotor;

  private final DifferentialDrive m_drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    super();
    m_frontRMotor = new CANSparkMax(DriveConstants.kRightFrontMotorPort, MotorType.kBrushless);
    m_frontLMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorPort, MotorType.kBrushless);
    m_backRMotor = new CANSparkMax(DriveConstants.kRightBackMotorPort, MotorType.kBrushless);
    m_backLMotor = new CANSparkMax(DriveConstants.kLeftBackMotorPort, MotorType.kBrushless);

    m_backLMotor.follow(m_frontLMotor);
    m_backLMotor.setInverted(false);
    m_backRMotor.follow(m_frontRMotor);

    m_drive = new DifferentialDrive(m_frontLMotor, m_frontRMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forward, double rotation) {
    //rotation = rotation * 0.875;
    m_drive.arcadeDrive(forward, rotation, true);
  }
}
