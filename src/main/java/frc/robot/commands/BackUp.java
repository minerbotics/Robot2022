package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BackUp extends SequentialCommandGroup {

  public BackUp(DriveTrain drivetrain) {
    addCommands(
      new AutoDrive(drivetrain, -1).withTimeout(1.2)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
}