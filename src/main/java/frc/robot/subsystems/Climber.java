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
import frc.robot.subsystems.DriveBase.DriveBaseLoggingData;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.Gyro;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public boolean m_isActive = true;
  static Climber m_Instance = null;
  public static boolean climbingStatus = false;
  private static WPI_TalonSRX m_leftFrontClimberMotor, m_rightFrontClimberMotor , m_leftBackClimberMotor, m_rightBackClimberMotor;
  private Solenoid m_leftClimberSolenoid, m_rightClimberSolenoid;
  public static boolean leftClimberState,  rightClimberState;
  private static double rightVoltage, leftVoltage, m_roll;
  private long m_lastLogTime;
  private ClimberLoggingData m_loggingData;
  private AsyncStructuredLogger<ClimberLoggingData> m_logger;

  public Climber() {
    if (m_isActive == false) {
      return;
    }
    /* Right Solenoid Set */ m_rightClimberSolenoid = new Solenoid(Constants.PCMCAN_Address, PneumaticsModuleType.CTREPCM, Constants.RightClimberChannel);
    /* Left Solenoid Set */ m_leftClimberSolenoid = new Solenoid(Constants.PCMCAN_Address, PneumaticsModuleType.CTREPCM, Constants.LeftClimberChannel);
    /* Right Front Motor Set */ m_rightFrontClimberMotor = new WPI_TalonSRX(Constants.RightFrontClimberMotorCAN_Address);
    /* Left Front Motor Set */ m_leftFrontClimberMotor = new WPI_TalonSRX(Constants.LeftFrontClimberMotorCAN_Address);
    /* Right Back Motor Set */ m_rightBackClimberMotor = new WPI_TalonSRX(Constants.RightWinchMotorCAN_Address);
    /* Left Back Motor Set */ m_leftBackClimberMotor = new WPI_TalonSRX(Constants.LeftWinchMotorCAN_Address);
    /* Right Front Motor Set Brake Mode */ m_rightFrontClimberMotor.setNeutralMode(NeutralMode.Brake);
    /* Left Front Motor Set Brake Mode */ m_leftFrontClimberMotor.setNeutralMode(NeutralMode.Brake);
    /* Right Back Motor Set Brake Mode */ m_rightBackClimberMotor.setNeutralMode(NeutralMode.Brake);
    /* Left Back Motor Set Brake Mode */ m_leftBackClimberMotor.setNeutralMode(NeutralMode.Brake);
    leftClimberState = false;
    rightClimberState = false;
    m_roll = Gyro.getRoll();
    m_loggingData = new ClimberLoggingData();
    m_logger = new AsyncStructuredLogger<ClimberLoggingData>("Climber", /*forceUnique=*/false, ClimberLoggingData.class);
  }

  public static Climber getInstance() {
    if(m_Instance == null) {
			synchronized (Climber.class) {
				m_Instance = new Climber();
			}
		}
		return m_Instance;
  }

  //Pull up with Back motors by set amount of rotations
/*   public void climbStep1(double rotations) {
    m_leftBackClimberMotor pull by set amount of rotations
    m_rightBackClimberMotor pull by set amount of rotations
    m_leftFrontClimberMotor.setNeutralMode(NeutralMode.Coast);
    m_rightFrontClimberMotor.setNeutralMode(NeutralMode.Coast);
  } */

  //Extend Pistons
/*   public void climbStep2() {
    m_leftClimberSolenoid.set(true);
    m_rightClimberSolenoid.set(true);
  } */

  //Pull down Front Motors until it hits high bar
/*   public void climbStep3(double rotations) {
    m_leftFrontClimberMotor pull by set amount of rotations
    m_rightFrontClimberMotor pull by set amount of rotations
    m_leftBackClimberMotor.setNeutralMode(NeutralMode.Coast);
    m_rightBackClimberMotor.setNeutralMode(NeutralMode.Coast);
  } */

  //Retract Pistons
/*   public void climbStep4() {
    m_leftClimberSolenoid.set(false);
    m_rightClimberSolenoid.set(false);
  } */

  // Left
    // Left Front Motor
      public double getLeftFrontEncoder() {
        return m_leftFrontClimberMotor.getSelectedSensorPosition();
      }

      public void setLeftFrontMotorLevel(double mL) {
        m_leftFrontClimberMotor.set(mL);
      }

      public void setLeftFrontVolts(double volts) {
        m_leftFrontClimberMotor.setVoltage(volts);
      }

      public double getLeftFrontVolts() {
        return m_leftFrontClimberMotor.getBusVoltage();
      }

      public double getLeftRotaions() {
        return (getLeftFrontEncoder() / 4096);
      }

  // Right
    // Right Front
      public double getRightFrontEncoder() {
        return m_rightFrontClimberMotor.getSelectedSensorPosition();
      }

      public void setRightFrontMotorLevel(double mL) {
        m_rightFrontClimberMotor.set(mL);
      }

      public void setRightFrontVolts(double volts) {
        m_rightFrontClimberMotor.setVoltage(volts);
      }

      public double getRightFrontVolts() {
        return m_rightFrontClimberMotor.getBusVoltage();
      }

      public double getRightRotations() {
        return (getRightFrontEncoder() / 4096);
      }

  public void toggleClimbPistons() {
    leftClimberState = !leftClimberState; //Reflects this change in a constant so we know where the piston is
    rightClimberState = !rightClimberState;
    m_leftClimberSolenoid.toggle(); //Sets the pistons to the opposite state of what they were
    m_rightClimberSolenoid.toggle();
  }

  public boolean getClimbingStatus() {
    return climbingStatus;
  }

  public void setClimbingStatus(boolean status) {
    climbingStatus = status;
  }


  public void winchUp() {
    setClimbingStatus(true);
    m_leftBackClimberMotor.set(0.75);
    m_rightBackClimberMotor.set(-0.75);
  }

  public void winchDown() {
    setClimbingStatus(true);
    m_leftBackClimberMotor.set(-0.75);
    m_rightBackClimberMotor.set(0.75);
  }

  public void winchStop() {
    setClimbingStatus(false);
    m_leftBackClimberMotor.set(0);
    m_rightBackClimberMotor.set(0);
  }

  private double getRateOfChange(double initialValue, double finalValue, long initialTime, long finalTime) {
    return (finalValue - initialValue) / (finalTime - initialTime);
  }

  @Override
  public void periodic() {
    if (m_isActive == false) {
      return;
    }

    // This method will be called once per scheduler run
    long now = System.nanoTime();
    double lastLeftEncoderReading = m_loggingData.LeftEncoderReading;
    double lastLeftVelocity = m_loggingData.LeftVelocity;
    double lastRightEncoderReading = m_loggingData.RightEncoderReading;
    double lastRightVelocity = m_loggingData.RightVelocity;

    Robot.getGyro();
    m_roll = Gyro.getYaw();
    SmartDashboard.putNumber("left climber", getLeftFrontEncoder());
    SmartDashboard.putNumber("right climber", getRightFrontEncoder());
    SmartDashboard.putBoolean("Pistons Extended", leftClimberState && rightClimberState);
    SmartDashboard.putNumber("Left Rotations", m_loggingData.LeftRotations);
    SmartDashboard.putNumber("Right Rotations", m_loggingData.RightRotations);
    SmartDashboard.putNumber("Left Climber Motor Voltage", leftVoltage);
    SmartDashboard.putNumber("Right Climber Motor Voltage", rightVoltage);
    SmartDashboard.putNumber("Left Climber Motor Level", m_leftFrontClimberMotor.get());
    SmartDashboard.putNumber("Right Climber Motor Level", m_rightFrontClimberMotor.get());
    SmartDashboard.putNumber("Left Climber Motor Current Out", m_leftFrontClimberMotor.getStatorCurrent());
    SmartDashboard.putNumber("Right Climber Motor Current Out", m_rightFrontClimberMotor.getStatorCurrent());
    SmartDashboard.putNumber("Left Climber Motor Current In", m_leftFrontClimberMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Right Climber Motor Current In", m_rightFrontClimberMotor.getSupplyCurrent());

    if (climbingStatus) {
      System.out.println("roll control");
      double climbDirection = OI.getInstance().getLeftClimbStick();
      leftVoltage = climbDirection * RobotController.getBatteryVoltage();
      rightVoltage = climbDirection * RobotController.getBatteryVoltage();
        
      double delta = Gyro.getRoll() - m_roll;

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
          }
          else if (delta > 0) {
            rightVoltage -= Constants.climberVoltageChange;
            leftVoltage += Constants.climberVoltageChange;
          }
        }
      }
      
        setLeftFrontVolts(leftVoltage);
        setRightFrontVolts(rightVoltage);
    }


    m_loggingData.LeftFrontMotorLevel = m_leftFrontClimberMotor.get();
    m_loggingData.RightFrontMotorLevel = m_rightFrontClimberMotor.get();
    m_loggingData.LeftEncoderReading = getLeftFrontEncoder();
    m_loggingData.RightEncoderReading = getRightFrontEncoder();
    m_loggingData.LeftVelocity = (getLeftFrontEncoder() - lastLeftEncoderReading) / (now - m_lastLogTime);
    m_loggingData.RightVelocity = (getRightFrontEncoder() - lastRightEncoderReading) / (now - m_lastLogTime);
    m_loggingData.LeftAcceleration = (getRateOfChange(lastLeftVelocity, m_loggingData.LeftVelocity, m_lastLogTime, now));
    m_loggingData.RightAcceleration = (getRateOfChange(lastRightVelocity, m_loggingData.RightVelocity, m_lastLogTime, now));
    m_loggingData.LeftRotations = getLeftRotaions();
    m_loggingData.RightRotations = getRightRotations();

    m_logger.queueData(m_loggingData);
    m_lastLogTime = now;
  }

  public static class ClimberLoggingData {
    public double LeftFrontMotorLevel, RightFrontMotorLevel;
    public double LeftEncoderReading, RightEncoderReading;
    public double LeftVelocity, RightVelocity;
    public double LeftAcceleration, RightAcceleration;
    public double LeftRotations, RightRotations;
  }
}
