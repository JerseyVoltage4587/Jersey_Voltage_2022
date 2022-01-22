// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoChoice extends CommandBase {
  /** Creates a new Auto. */
    File file = (new File("frc.robot.AutoCommandChoice"));
    static String autoChoice;

  public AutoChoice() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Scanner r = new Scanner(file);
      autoChoice = r.next();
      r.close();
    } 

    catch (FileNotFoundException e) {
      System.out.println(e);
    }
  }

  public static Command getAutoChoice() {
    switch (autoChoice) {
      case "AutoCommand_1":
        return (new AutoCommand_1());

      case "AutoCommand_2":
        return (new AutoCommand_2());
    }
    return null;
  }

  // Called every time the scheduler runs while the command is scheduled.it
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
