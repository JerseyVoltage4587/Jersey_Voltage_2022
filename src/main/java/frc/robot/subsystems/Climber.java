// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.util.Gyro;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public boolean m_isActive = true;
  static Climber m_Instance = null;
  public static boolean climbingStatus = false;
  private static WPI_TalonSRX m_leftClimberMotor, m_rightClimberMotor;
  private Solenoid m_leftClimberSolenoid, m_rightClimberSolenoid;
  public static boolean leftClimberState,  rightClimberState;
  private static double rightVoltage, leftVoltage, m_yaw;

  public Climber() {
    if (m_isActive == false) {
      return;
    }
    m_leftClimberSolenoid = new Solenoid(Constants.PCMCAN_Address, PneumaticsModuleType.CTREPCM, Constants.LeftClimberChannel);
    m_rightClimberSolenoid = new Solenoid(Constants.PCMCAN_Address, PneumaticsModuleType.CTREPCM, Constants.RightClimberChannel);
    m_rightClimberMotor = new WPI_TalonSRX(Constants.RightClimberMotorCAN_Address);
    m_leftClimberMotor = new WPI_TalonSRX(Constants.LeftClimberMotorCAN_Address);
    m_rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putString("right brake", "right");
    m_leftClimberMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putString("left brake", "left");
    leftClimberState = false;
    rightClimberState = false;
  }

  public static Climber getInstance() {
    if(m_Instance == null) {
			synchronized (Climber.class) {
				m_Instance = new Climber();
			}
		}
		return m_Instance;
  }

  //Unspool winches by set amount
  public void climbStep1(double rotations) {
    //m_leftClimberMotor
    //m_rightClimberMotor
  }

  public double getLeftEncoder() {
    return m_leftClimberMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return m_rightClimberMotor.getSelectedSensorPosition();
  }

  public void setRightMotorVolts(double volts) {
    /* m_rightClimberMotor.setVoltage(volts); */
  }

  public void setLeftMotorVolts(double volts) {
    /* m_leftClimberMotor.setVoltage(volts); */
  }

  //Deploy pistons to lock
  public void climbStep2() {
    /* m_leftClimberSolenoid.set(true);
    m_rightClimberSolenoid.set(true); */
  }

  //Retract pistons
  public void climbStep4() {
    /* m_leftClimberSolenoid.set(false);
    m_rightClimberSolenoid.set(false); */
  }

  public void toggleClimbPistons() {
    return;
    /* leftClimberState = !leftClimberState; //Reflects this change in a constant so we know where the piston is
    rightClimberState = !rightClimberState;
    m_leftClimberSolenoid.set(leftClimberState); //Sets the pistons to the opposite state of what they were
    m_rightClimberSolenoid.set(rightClimberState); */
  }

  public boolean getClimbingStatus() {
    return climbingStatus;
  }

  public void toggleClimbMotors() {
    return;
    /* climbingStatus = !climbingStatus; */
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Robot.getGyro();
    m_yaw = Gyro.getYaw();
    SmartDashboard.putNumber("left climber", getLeftEncoder());
    SmartDashboard.putNumber("right climber", getRightEncoder());
    /* if (climbingStatus) {
      double climbDirection = OI.getInstance().getClimb();
      leftVoltage = climbDirection * RobotController.getBatteryVoltage();
      rightVoltage = climbDirection * RobotController.getBatteryVoltage();
        
      double delta = Gyro.getYaw() - m_yaw;

      if (delta != 0) {
        if (delta > 180) {
          delta -= 360;
        }

        if (delta < -180) {
          delta += 360;
        }

        if (Math.abs(delta) > 1) {
          // "I think we should not only alter right motor voltage but also left
          //so the case doesn't happen where one side is as increased as it can get and stuff still isn't fixed"
          if (delta < 0) {
            rightVoltage += Constants.climberVoltageChange;
            leftVoltage -= Constants.climberVoltageChange;
            setRightMotorVolts(rightVoltage);
          }
          else if (delta > 0) {
            rightVoltage -= Constants.climberVoltageChange;
            leftVoltage += Constants.climberVoltageChange;
            setRightMotorVolts(rightVoltage);
          }
        }
      }
      
        setLeftMotorVolts(leftVoltage);
        setRightMotorVolts(rightVoltage);
    } */
  }
}
