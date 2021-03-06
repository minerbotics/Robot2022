// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.FeedIn;
import frc.robot.commands.FeedOut;
import frc.robot.commands.FeedStop;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.ScoreTwice;
import frc.robot.commands.StopArm;
import frc.robot.commands.WaitForwardDumpBack;
import frc.robot.commands.DumpAndBackUp;
import frc.robot.commands.BackUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain;
  private final Arm m_arm;
  private final Intake m_intake;

  private final XboxController m_xboxController;

  private final FeedIn m_inCommand;
  private final FeedOut m_outCommand;
  private final FeedStop m_feedStopCommand;
  private final LowerArm m_lowerArmCommand;
  private final RaiseArm m_raiseArmCommand;
  private final StopArm m_stopArmCommand;

  private final DumpAndBackUp m_dumpAndBackUpCommand;
  private final BackUp m_backUpCommand;
  private final WaitForwardDumpBack m_waitDumpBackCommand;
  private final ScoreTwice m_scoreTwice;

  public static SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain = new DriveTrain();
    m_arm = new Arm();
    m_intake = new Intake();

    m_raiseArmCommand = new RaiseArm(m_arm);
    m_lowerArmCommand = new LowerArm(m_arm);
    m_stopArmCommand = new StopArm(m_arm);

    m_inCommand = new FeedIn(m_intake);
    m_outCommand = new FeedOut(m_intake);
    m_feedStopCommand = new FeedStop(m_intake);

    m_dumpAndBackUpCommand = new DumpAndBackUp(m_driveTrain, m_intake);
    m_backUpCommand = new BackUp(m_driveTrain);
    m_waitDumpBackCommand = new WaitForwardDumpBack(m_driveTrain, m_intake);
    m_scoreTwice = new ScoreTwice(m_driveTrain, m_intake, m_arm);

    m_xboxController = new XboxController(IOConstants.kDriverControllerPort);

    m_driveTrain.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new RunCommand(
          () ->
              m_driveTrain.arcadeDrive(
                -m_xboxController.getLeftY(), m_xboxController.getRightX()),
          m_driveTrain));

    // Configure the button bindings
    configureButtonBindings();

    m_chooser = new SendableChooser<Command>();
    m_chooser.setDefaultOption("DumpAndBackUp", m_dumpAndBackUpCommand);
    m_chooser.addOption("BackUp", m_backUpCommand);
    m_chooser.addOption("WaitDumpBack", m_waitDumpBackCommand);
    m_chooser.addOption("ScoreTwice", m_scoreTwice);

    SmartDashboard.putData(m_chooser);

    CameraServer.startAutomaticCapture();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_xboxController, IOConstants.kYButton).whenPressed(m_raiseArmCommand).whenReleased(m_stopArmCommand);
    new JoystickButton(m_xboxController, IOConstants.kAButton).whenPressed(m_lowerArmCommand).whenReleased(m_stopArmCommand);
    new JoystickButton(m_xboxController, IOConstants.kRBButton).whenPressed(m_inCommand).whenReleased(m_feedStopCommand);
    new JoystickButton(m_xboxController, IOConstants.kLBButton).whenPressed(m_outCommand).whenReleased(m_feedStopCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
