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
    public static final int LeftFrontClimberMotorCAN_Address = 55; //final
    public static final int RightFrontClimberMotorCAN_Address = 56; //final
    public static final int LeftBackClimberMotorCAN_Address = 35; //final
    public static final int RightBackClimberMotorCAN_Address = 36; //final
    public static final int PCMCAN_Address = 12; //final
    public static final int LeftWinchMotorCAN_Address = 17;
    public static final int RightWinchMotorCAN_Address = 18;



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
    public static final int LeftFrontClimberMotorPDP_Port = 2; //final
    public static final int RightFrontClimberMotorPDP_Port = 13; //final
    public static final int LeftWinchMotorPDP_Port = 9;
    public static final int RightWinchMotorPDP_Port = 10;
    // public static final int LeftBackClimberMotorPDP_Port = ; //TBD
    // public static final int RightBackClimberMotorPDP_Port = ; //TBD
    
    //Solenoid IDs
    //public static final int Module_ID = 0;

    //Solenoid Channel
    public static final int LeftIntakeChannel = 4; //CHANGE THIS IS TEMPORARY 
    public static final int RightIntakeChannel = 5; //CHANGE THIS IS TEMPORARY 

    public static final int LeftClimberChannel = 2; //CHANGE THIS IS TEMPORARY 
    public static final int RightClimberChannel = 3; //CHANGE THIS IS TEMPORARY 

    //Motor Levels
    public static double IntakeMotorLevel = 0.85;
    public static double IntakeBackMotorLevel = 0.7;
    public static double ShooterMotorLowRPM = 1650; //LOW GOAL RPM
    public static double ShooterMotorHighRPM = 4200; //HIGH GOAL RPM
    public static double ShooterMotorPadRPM = 5400; //LAUNCHPAD GOAL RPM
    public static double ShooterBackMotorRPM = -60; //TBD
    public static double StorageMotorLevel = 0.75;
    public static double StorageMotorLevelShooting = 0.6; //.5
    public static double StorageBackMotorLevel = -0.5;
    public static double KickerMotorLevel = -0.8; //TBD
    public static double KickerSlowBackMotorLevel = 0.4; //TBD
    public static double KickerFastBackMotorLevel = 0.7; //TBD
    
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
/*        public static final double ksVoltsLeft = 0;
        public static final double kvVoltsLeft = 0;
        public static final double kaVoltsLeft = 0;*/
        public static final double kpDriveVel = 0;
/*        public static final double ksVoltsRight = 0;
        public static final double kvVoltsRight = 0;
        public static final double kaVoltsRight = 0;*/
        // Unit of gain uncertain - likely meter, possibly rotations.
        public static final double ksVoltsShooter = 0.5;//0.89343;
        public static final double kvVoltsShooter = 0.00142;//0.13196;
        public static final double kaVoltsShooter = 0.0016662;
    public final static double ksVolts = 0; //TBD
    public final static double kvVolts = 0; //TBD
    public final static double kaVolts = 0; //TBD

    //Modes
    public final static int IntakeOFF_MODE = 0;
    public final static int IntakeIN_MODE = 1;
    public final static int IntakeOUT_MODE = 2;
    public final static int ShooterON_MODE = 0;
    public final static int ShooterOFF_MODE = 1;
    public final static int ShooterBACK_MODE = 2;
    public final static int FrontCamera = 0; //front camera
    public final static int BackCamera = 1; //back camera

    public final static double climberVoltageChange = 0.001;
}