package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BackUp extends SequentialCommandGroup {

  public BackUp(DriveTrain drivetrain) {
    addCommands(
      new AutoDrive(drivetrain, -0.5).withTimeout(1.5)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
}