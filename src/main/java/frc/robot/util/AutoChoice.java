// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto.Auto;
import frc.robot.commands.Auto.TwoBallAuto;

public class AutoChoice {
  /** Creates a new Auto. */
    static File file = (new File("frc.robot.AutoCommandChoice"));
    static String autoChoice = "";
    public static HashMap<String, Command> autoCommands = new HashMap<String, Command>();

  public AutoChoice() {
    // Use addRequirements() here to declare subsystem dependencies.
    autoCommands.put("TwoBallAuto", new TwoBallAuto());
    autoCommands.put("Auto", new Auto(75, 0));
  }

  public static Command getAutoChoice() {
    try {
        Scanner r = new Scanner(file);
        autoChoice = r.next();
        r.close();
    }
  
    catch (FileNotFoundException e) {
      System.out.println(e);
    }

    return autoCommands.get(autoChoice);
  }
}
