// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeBall;
import frc.robot.commands.Intake.StopIntakeBall;
import frc.robot.commands.Shooter.PrepareShooter;
import frc.robot.commands.Shooter.ShootBall;
import frc.robot.commands.Shooter.StopShootBall;
import frc.robot.commands.Shooter.StopShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallHighRight extends SequentialCommandGroup {
  /** Creates a new TwoBallHighRight. */
  public TwoBallHighRight(int distance, double heading) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PrepareShooter(Constants.ShooterMotorHighRPM), 
          new IntakeBall(), 
          new AutoForward(distance, heading), 
          new WaitCommand(1), new StopIntakeBall(),  
          new WaitCommand(0.5),
          new AutoBack(10, heading),
          new WaitCommand(1), 
          new ShootBall(), new WaitCommand(0.125), new StopShootBall(), new WaitCommand(1), new ShootBall(),
          new WaitCommand(2), 
          new AutoForward(8, heading), 
          new StopShootBall(), 
          new StopShooter()
      );
  }
}
