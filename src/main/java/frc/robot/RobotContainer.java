// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto.TwoBallHigh;
import frc.robot.commands.Auto.TwoBallHighRight;
import frc.robot.commands.Auto.TwoBallLow;
import frc.robot.commands.Auto.TwoBallLowRight;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.Auto.OneBallHigh;
import frc.robot.commands.Auto.OneBallLow;
import frc.robot.commands.Auto.SimpleAutoForward;
import frc.robot.commands.Auto.AutoForward;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */




public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser = new SendableChooser<>();
    // Configure the button bindings
    Robot.autoWaitTime = SmartDashboard.getNumber("Auto Wait Time", 0);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_chooser.setDefaultOption("Two Ball (High)", new TwoBallHigh(90, 0));
    m_chooser.addOption("Leave Tarmac", new SimpleAutoForward(85, 0));
    m_chooser.addOption("Do Nothing", new DoNothing());
    m_chooser.addOption("Two Ball (Low)", new TwoBallLow(85, 0));
    m_chooser.addOption("One Ball (High)", new OneBallHigh());
    m_chooser.addOption("One Ball (Low)", new OneBallLow());
    m_chooser.addOption("Two Ball High Right", new TwoBallHighRight(40, 0));
    m_chooser.addOption("Two Ball Low Right", new TwoBallLowRight(20, 0));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
