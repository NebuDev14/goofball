// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  private final Joystick buttons = new Joystick(2);

  private final Autos autos = new Autos(m_drivetrain);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // schedule the auto, as god intended
    new Trigger(DriverStation::isAutonomous)
        .whileTrue(Commands.deferredProxy(m_chooser::getSelected));

    m_drivetrain.setDefaultCommand(
        m_drivetrain.tankDrive(() -> left.getRawAxis(0), () -> right.getRawAxis(0)));

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Back and Forth", autos.backAndForth());
    m_chooser.addOption("Test Trajectory", autos.testTrajectory());
    SmartDashboard.putData(m_chooser);
  }
}
