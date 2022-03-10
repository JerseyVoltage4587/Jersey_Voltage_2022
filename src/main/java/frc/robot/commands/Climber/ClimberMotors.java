// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.util.Gyro;

public class ClimberMotors extends CommandBase {
  private final Climber m_climber;
  private final double m_rotations, m_tolerance;
  private final TrapezoidProfile m_profile;
  private double m_startTime, m_startLeftRotations, m_startRightRotations;
  private double m_leftRotations, m_rightRotations;
  private boolean m_forward;
  private double left_voltage, right_voltage;
  private double m_yaw;
  
public ClimberMotors(double distance, double tolerance, boolean forward) {
    m_rotations   = Math.abs(Units.feetToMeters(distance));    // meters
    m_tolerance  = Units.inchesToMeters(tolerance);   // meters
    m_climber = Robot.getClimber();
    m_forward = forward;
    m_profile = new TrapezoidProfile (
                    new TrapezoidProfile.Constraints(Constants.kMaxSpeed,
                                                     Constants.kMaxAcceleration),
                    new TrapezoidProfile.State(m_rotations,0),
                    new TrapezoidProfile.State(0,0)
                );
    addRequirements(m_climber);
    System.out.println("m_rotations="+m_rotations+",m_tolerance="+m_tolerance+",m_profile.totalTime="+m_profile.totalTime());
  }

  @Override
  public void initialize() {
    m_startTime        = Timer.getFPGATimestamp(); // Get start time
    m_startLeftRotations  = Units.inchesToMeters(m_climber.getLeftEncoder()); // get distance for left
    m_startRightRotations = Units.inchesToMeters(m_climber.getRightEncoder()); // get distance for right
    Robot.getGyro();
    m_yaw = Gyro.getYaw();
    System.out.println("m_startTime="+m_startTime+",m_startLeft="+m_startLeftRotations+",m_startRight="+m_startRightRotations);
  }

  @Override
  public void execute() {
    double elapsed_time = Timer.getFPGATimestamp() - m_startTime; // subtracts startTime from timer to get more accurate time.
    m_leftRotations  = Math.abs(Units.inchesToMeters(m_climber.getLeftEncoder())  - m_startLeftRotations);
      // subtracts starting distance from distance to get more accurate distance.
    m_rightRotations = Math.abs(Units.inchesToMeters(m_climber.getRightEncoder()) - m_startRightRotations);
      // subtracts starting distance from distance to get more accurate distance.

    double expected_distance, expected_velocity, expected_acceleration;
    if ( elapsed_time > m_profile.totalTime()) { // when the time passes the expected time
        expected_distance     = m_rotations; // set expected distance to the distance inputted to travel
        expected_velocity     = 0; // set expected velocity to 0
        expected_acceleration = 0; // set expected acceleration to 0
    }
    else {
        TrapezoidProfile.State expected_state = m_profile.calculate(elapsed_time); // calculated the current expected state
        TrapezoidProfile.State next_state     = m_profile.calculate(elapsed_time + Constants.kSecondsPerCycle);
          // calculate the expected state in the next cycle (0.02s)
        expected_distance     = (expected_state.position); // set expected distance to the position of the current state
        expected_velocity     = expected_state.velocity; // set expected velocity to the velocity of the current state
        expected_acceleration = (next_state.velocity - expected_state.velocity) / Constants.kSecondsPerCycle; // 


    }

    double left_error  = Math.abs(expected_distance - m_leftRotations);
    double right_error = Math.abs(expected_distance - m_rightRotations);
    if (m_forward == true) {
      left_voltage = Constants.ksVoltsLeft
                                + expected_velocity * Constants.kvVoltsLeft
                                + expected_acceleration * Constants.kaVoltsLeft
                                + left_error * Constants.kpDriveVel;

      right_voltage = Constants.ksVoltsRight
                                + expected_velocity * Constants.kvVoltsRight
                                + expected_acceleration * Constants.kaVoltsRight
                                + right_error * Constants.kpDriveVel;
      
    } 
    else {
      left_voltage = -1 * (Constants.ksVoltsLeft
                                + expected_velocity * Constants.kvVoltsLeft
                                + expected_acceleration * Constants.kaVoltsLeft
                                + left_error * Constants.kpDriveVel);

      right_voltage = -1 * (Constants.ksVoltsRight
                                + expected_velocity * Constants.kvVoltsRight
                                + expected_acceleration * Constants.kaVoltsRight
                                + right_error * Constants.kpDriveVel);
    }

    if (Math.abs(m_leftRotations - m_rightRotations) > m_tolerance) {
        
    }
    

    
    Robot.getGyro();
    double delta = Gyro.getYaw() - m_yaw;

    if (delta != 0) {
      if (delta > 180) {
        delta -= 360;
      }

      if (delta < -180) {
        delta += 360;
      }

      if (Math.abs(delta) > 1) { //I think we should not only alter right motor voltage but also left so the case doesn't happen where one side is increased as it can get and stuff still isn't fixed
        if (delta < 0) {
          right_voltage += 0.001; //0.001 should be a constant
          left_voltage -= 0.001;
          m_climber.setRightMotorVolts(right_voltage);
        }
        else if (delta > 0) {
          right_voltage -= 0.001;
          left_voltage += 0.001;
          m_climber.setRightMotorVolts(right_voltage);
        }
      }
    }
    
      m_climber.setLeftMotorVolts(left_voltage);
      m_climber.setRightMotorVolts(right_voltage);


    System.out.println("left voltage: " + left_voltage);
    System.out.println("right voltage: " + right_voltage);
    SmartDashboard.putNumber("Expected Distance", expected_distance);
    SmartDashboard.putNumber("Expected Velocity", expected_velocity);
    SmartDashboard.putNumber("Actual Velocity", ((m_leftRotations + m_rightRotations) / 2)  /elapsed_time);
    SmartDashboard.putNumber("Expected Acceleration", expected_acceleration);
    SmartDashboard.putNumber("Left Travel", m_leftRotations);
    SmartDashboard.putNumber("Right Travel", m_rightRotations);
    SmartDashboard.putNumber("Left Error", left_error);
    SmartDashboard.putNumber("Right Error", right_error);
    SmartDashboard.putNumber("Left Voltage", left_voltage);
    SmartDashboard.putNumber("Right Voltage", right_voltage);
    SmartDashboard.putNumber("Left Shortage", m_leftRotations - m_rotations);
    SmartDashboard.putNumber("Right Shortage", m_rightRotations - m_rotations);
    SmartDashboard.putNumber("Elapsed Time", elapsed_time);
    SmartDashboard.putNumber("Expected Completion Time", m_profile.totalTime());
    SmartDashboard.putNumber("Average Travel", (m_leftRotations + m_rightRotations)/2);
  }

  @Override
  public void end(boolean interrupted) {

        // Let the drivetrain revert to its default command.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_leftRotations  - m_rotations) < m_tolerance)
           &&
           (Math.abs(m_rightRotations - m_rotations) < m_tolerance);
  }
}
