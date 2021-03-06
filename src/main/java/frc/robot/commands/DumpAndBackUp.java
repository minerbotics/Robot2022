package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DumpAndBackUp extends SequentialCommandGroup {

  public DumpAndBackUp(DriveTrain drivetrain, Intake intake) {
    addCommands(
      new FeedStop(intake).withTimeout(3),
      new FeedOut(intake).withTimeout(1),
      new FeedStop(intake).withTimeout(0.1),
      new AutoDrive(drivetrain, -1).withTimeout(1.5)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake);
  }
}