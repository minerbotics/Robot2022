package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreTwice extends SequentialCommandGroup {

  public ScoreTwice(DriveTrain drivetrain, Intake intake, Arm arm) {
    addCommands(
      new FeedOut(intake).withTimeout(1),
      new FeedStop(intake).withTimeout(0.1),
      new AutoDrive(drivetrain, -1).withTimeout(1.5),
      new AutoRotate(drivetrain, 1).withTimeout(0.5),
      new LowerArm(arm).withTimeout(0.1),
      new FeedIn(intake).withTimeout(1),
      new AutoDrive(drivetrain, 0.625).withTimeout(0.4),
      new FeedStop(intake).withTimeout(0.1),
      new AutoDrive(drivetrain, -0.625).withTimeout(0.4),
      new RaiseArm(arm).withTimeout(1),
      new AutoRotate(drivetrain, -1).withTimeout(0.5),
      new AutoDrive(drivetrain, 1).withTimeout(1.5),
      new FeedOut(intake).withTimeout(1)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake, arm);
  }
}