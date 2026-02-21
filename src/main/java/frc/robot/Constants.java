// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

  //Motor Controller Configs for different Amp Allowances
  public static final SparkMaxConfig kTwntyAmp = new SparkMaxConfig();
  public static final SparkMaxConfig kFortyAmp = new SparkMaxConfig();
  public static final SparkMaxConfig kThirtyAmp = new SparkMaxConfig();
  
    public static final class OperatorConstants{

        //Controller Joystick Deadzone
        public static final double Deadzone = 0.15;
        //Limelight Targeting Allowance
        public static final double limelightDeadzone = 0.05;
      }
    
    public static final class DriveConstants{
    //Maximum speed allowance for Drivetrain
    public static final double maximumSpeed = Units.feetToMeters(13.59);
    
    //Motor type shortcut for SparkMAX set up.
    public static final MotorType NEO = MotorType.kBrushless;
    public static final MotorType NEO550 = MotorType.kBrushless;
    }

    public static final class LimelightConstants{
      //Red Hub Apriltag ID's (will need to be changed later)
      public static final int kRedHubFTMid = 1;
      public static final int kRedHubRTMid = 2;
      public static final int kRedHubLTMid = 3;
      //Blue Hub Apriltag ID's (will need to be changed later)
      public static final int kBlueHubFTMid = 4;
      public static final int kBlueHubRTMid = 5;
      public static final int kBlueHubLTMid = 6;
    }

    public static final class TurretConstants{
      public static final double kHoodHome = 0;
      public static final double kHoodMax = 100; //number will need to be adjusted later
      public static final double kHoodPass = 50; //number will need to be adjusted later
      public static final double kTurRotHome = 0;
      public static final double kTurLTMax = 100; //number will need to be changed later
      public static final double kTurRTMax = -100; //number will need to be changed later
    }

    static{
      //Forty Amp Limit
      kFortyAmp.smartCurrentLimit(45);
      //Thirty Amp Limit
      kThirtyAmp.smartCurrentLimit(35);
      //Twenty Amp Limit
      kTwntyAmp.smartCurrentLimit(25);

    
    }
}

