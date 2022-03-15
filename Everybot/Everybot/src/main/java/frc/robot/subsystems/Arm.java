// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_armMotor;

  /** Creates a new Arm. */
  public Arm() {
    m_armMotor = new CANSparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
  }

  public void raise() {
    m_armMotor.set(0.2);
  }

  public void stop() {
    m_armMotor.set(0);
  }

  public void lower() {
    m_armMotor.set(-0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
