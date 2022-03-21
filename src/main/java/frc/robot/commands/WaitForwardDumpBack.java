package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class WaitForwardDumpBack extends SequentialCommandGroup {

  public WaitForwardDumpBack(DriveTrain drivetrain, Intake intake) {
    addCommands(
      new FeedStop(intake).withTimeout(8),
      new AutoDrive(drivetrain, .5).withTimeout(1),
      new FeedOut(intake).withTimeout(1),
      new FeedStop(intake).withTimeout(0.1),
      new AutoDrive(drivetrain, -1).withTimeout(1.5)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake);
  }
}