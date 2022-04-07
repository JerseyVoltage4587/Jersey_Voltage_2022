// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shooter.PrepareShooter;
import frc.robot.commands.Shooter.ShootBall;
import frc.robot.commands.Shooter.StaggerShootBall;
import frc.robot.commands.Shooter.StopShootBall;
import frc.robot.commands.Shooter.StopShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallHigh extends SequentialCommandGroup {
  /** Creates a new OneBallHigh. */
  public OneBallHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PrepareShooter(Constants.ShooterMotorHighRPM), new WaitCommand(2), new SimpleAutoForward(80, 0), new StaggerShootBall(), new WaitCommand(2), new SimpleAutoForward(10, 0), new StopShooter());
  }
}
