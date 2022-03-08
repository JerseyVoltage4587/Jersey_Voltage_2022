// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;

public class TrapezoidProfileMovement extends CommandBase {
  private final DriveBase m_drivetrain;
  private final double m_distance, m_tolerance;
  private final TrapezoidProfile m_profile;
  private double m_startTime, m_startLeftMeters, m_startRightMeters;
  private double m_leftTravel, m_rightTravel;
  private boolean m_forward;
  private double left_voltage, right_voltage;
public TrapezoidProfileMovement(double distance, double tolerance, boolean forward) {
    m_distance   = Math.abs(Units.feetToMeters(distance));    // meters
    m_tolerance  = Units.inchesToMeters(tolerance);   // meters
    m_drivetrain = Robot.getDriveBase();
    m_forward = forward;
    m_profile = new TrapezoidProfile (
                    new TrapezoidProfile.Constraints(Constants.kMaxSpeed,
                                                     Constants.kMaxAcceleration),
                    new TrapezoidProfile.State(m_distance,0),
                    new TrapezoidProfile.State(0,0)
                );
    addRequirements(m_drivetrain);
    System.out.println("m_distance="+m_distance+",m_tolerance="+m_tolerance+",m_profile.totalTime="+m_profile.totalTime());
  }

  @Override
  public void initialize() {
    m_startTime        = Timer.getFPGATimestamp(); // Get start time
    m_startLeftMeters  = Units.inchesToMeters(m_drivetrain.getLeftDistanceInches()); // get distance for left
    m_startRightMeters = Units.inchesToMeters(m_drivetrain.getRightDistanceInches()); // get distance for right
    System.out.println("m_startTime="+m_startTime+",m_startLeft="+m_startLeftMeters+",m_startRight="+m_startRightMeters);
  }

  @Override
  public void execute() {
    double elapsed_time = Timer.getFPGATimestamp() - m_startTime; // subtracts startTime from timer to get more accurate time.
    m_leftTravel  = Math.abs(Units.inchesToMeters(m_drivetrain.getLeftDistanceInches())  - m_startLeftMeters);
      // subtracts starting distance from distance to get more accurate distance.
    m_rightTravel = Math.abs(Units.inchesToMeters(m_drivetrain.getRightDistanceInches()) - m_startRightMeters);
      // subtracts starting distance from distance to get more accurate distance.

    double expected_distance, expected_velocity, expected_acceleration;
    if ( elapsed_time > m_profile.totalTime()) { // when the time passes the expected time
        expected_distance     = m_distance; // set expected distance to the distance inputted to travel
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

    double left_error  = Math.abs(expected_distance - m_leftTravel);
    double right_error = Math.abs(expected_distance - m_rightTravel);
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

    if (Math.abs(m_leftTravel - m_rightTravel) > m_tolerance) {
        
    }
    /*double delta = m_drivetrain.getGyroAngleZ() - m_heading;
    if (delta != 0) {
      if (delta > 180) {
        delta -= 360;
      }

      if (delta < -180) {
        delta += 360;
      }

      if (Math.abs(delta) > 1) {
        if (delta < 0) {
          right_voltage += 0.001;
          m_drivetrain.setRightVolts(right_voltage);
        }
        else if (delta > 0) {
          right_voltage -= 0.001;
          m_drivetrain.setRightVolts(right_voltage);
        }
      }
    }*/
    
      m_drivetrain.setLeftVolts  (left_voltage);
      m_drivetrain.setRightVolts (right_voltage);


    System.out.println("left voltage: " + left_voltage);
    System.out.println("right voltage: " + right_voltage);
    SmartDashboard.putNumber("Expected Distance", expected_distance);
    SmartDashboard.putNumber("Expected Velocity", expected_velocity);
    SmartDashboard.putNumber("Actual Velocity", ((m_leftTravel + m_rightTravel) / 2)  /elapsed_time);
    SmartDashboard.putNumber("Expected Acceleration", expected_acceleration);
    SmartDashboard.putNumber("Left Travel", m_leftTravel);
    SmartDashboard.putNumber("Right Travel", m_rightTravel);
    SmartDashboard.putNumber("Left Error", left_error);
    SmartDashboard.putNumber("Right Error", right_error);
    SmartDashboard.putNumber("Left Voltage", left_voltage);
    SmartDashboard.putNumber("Right Voltage", right_voltage);
    SmartDashboard.putNumber("Left Shortage", m_leftTravel - m_distance);
    SmartDashboard.putNumber("Right Shortage", m_rightTravel - m_distance);
    SmartDashboard.putNumber("Elapsed Time", elapsed_time);
    SmartDashboard.putNumber("Expected Completion Time", m_profile.totalTime());
    SmartDashboard.putNumber("Average Travel", (m_leftTravel + m_rightTravel)/2);
  }

  @Override
  public void end(boolean interrupted) {

        // Let the drivetrain revert to its default command.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_leftTravel  - m_distance) < m_tolerance)
           &&
           (Math.abs(m_rightTravel - m_distance) < m_tolerance);
  }
}
