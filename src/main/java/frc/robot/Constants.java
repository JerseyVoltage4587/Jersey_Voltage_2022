/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Auto
    public static final int backOffStartAmount = 4; // TBD

    //CAN Addresses
    public static final int RightSpark2CAN_Address = 2; //final
    public static final int RightSpark21CAN_Address = 21; //final
    public static final int LeftSpark1CAN_Address = 1; //final
    public static final int LeftSpark11CAN_Address = 11; //final
    public static final int LeftShooterMotorCAN_Address = 45; //final
    public static final int RightShooterMotorCAN_Address = 46; //final
    public static final int StorageCAN_Address = 3; //final
    public static final int IntakeMotorCAN_Address = 4; //final
    public static final int KickerMotorCAN_Address = 5; //final
    public static final int LeftClimberMotorCAN_Address = 55; //final
    public static final int RightClimberMotorCAN_Address = 56; //final
    public static final int PCMCAN_Address = 12; //CHANGE THIS IS TEMPORARY 

    //PDP Ports
    public static final int RightSpark2PDP_Port = 15; //final
    public static final int RightSpark21PDP_Port = 14; //final
    public static final int LeftSpark1PDP_Port = 0; //final
    public static final int LeftSpark11PDP_Port = 1; //final
    public static final int LeftShooterMotorPDP_Port = 13; //final
    public static final int RightShooterMotorPDP_Port = 3; //final
    public static final int StoragePDP_Port = 4; //final
    public static final int IntakeMotorPDP_Port = 12; //final
    public static final int KickerMotorPDP_Port = 5; //final
    public static final int LeftClimberMotorPDP_Port = 2; //final
    public static final int RightClimberMotorPDP_Port = 13; //final
    
    //Solenoid IDs
    //public static final int Module_ID = 0;

    //Solenoid Channel
    public static final int LeftIntakeChannel = 4; //CHANGE THIS IS TEMPORARY 
    public static final int RightIntakeChannel = 5; //CHANGE THIS IS TEMPORARY 

    public static final int LeftClimberChannel = 2; //CHANGE THIS IS TEMPORARY 
    public static final int RightClimberChannel = 3; //CHANGE THIS IS TEMPORARY 

    //Motor Levels
    public static double IntakeMotorLevel = -0.85;
    public static double IntakeBackMotorLevel = 0.5;
    public static double ShooterMotorRPM = -1500; //not final
    public static double ShooterBackMotorRPM = -600; //not final
    public static double StorageMotorLevel = 0.7;
    public static double StorageBackMotorLevel = -0.4;
    public static double KickerMotorLevel = 0.8; //not final
    public static double KickerSlowBackMotorLevel = 0.4; //not final
    public static double KickerFastBackMotorLevel = 0.7; //not final
    
    //Motor Stall Currents
    
    //Encoder Tics
    public static final int DriveBaseEncoderTics = 4096; //final
    
    //Wheel Diameters
    public static final double DriveBaseWheelDiameter = 6; //Inches //final
    public static final double ShooterWheelDiameter = 0; //Inches //TBD
    
    //Rotations
    public static final double ClimberForwardRotations = 0; //TBD
    public static final double ClimberBackwardRotations = 0; //TBD

    //Tolerance
    public static final double TrapezoidProfileTolerance = 1; //final

    //PID
        //Shooter
        public static final double kMaxSpeed = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kSecondsPerCycle = 0;
        public static final int ksVoltsLeft = 0;
        public static final double kvVoltsLeft = 0;
        public static final double kaVoltsLeft = 0;
        public static final double kpDriveVel = 0;
        public static final int ksVoltsRight = 0;
        public static final double kvVoltsRight = 0;
        public static final double kaVoltsRight = 0;

    //Modes
    public final static int IntakeOFF_MODE = 0;
    public final static int IntakeIN_MODE = 1;
    public final static int IntakeOUT_MODE = 2;
    public final static int ShooterON_MODE = 0;
    public final static int ShooterOFF_MODE = 1;
    public final static int ShooterBACK_MODE = 2;

    public final static double climberVoltageChange = 0.001;
}