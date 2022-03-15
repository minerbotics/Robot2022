package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DumpAndBackUp extends SequentialCommandGroup {

  public DumpAndBackUp(DriveTrain drivetrain, Intake intake) {
    addCommands(
      new FeedOut(intake).withTimeout(1.5),
      new AutoDrive(drivetrain, -0.5).withTimeout(1.5)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake);
  }
}