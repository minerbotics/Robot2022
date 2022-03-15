package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive extends CommandBase {
  private DriveTrain m_drivetrain;
  private double m_speed;

  public AutoDrive(DriveTrain drivetrain, double speed) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}