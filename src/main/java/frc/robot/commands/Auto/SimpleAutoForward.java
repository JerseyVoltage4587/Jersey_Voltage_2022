// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAutoForward extends SequentialCommandGroup {
  /** Creates a new SimpleAutoForward. */
  public SimpleAutoForward(int distance, double heading) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)), new AutoForward(distance, heading));
  }
}
