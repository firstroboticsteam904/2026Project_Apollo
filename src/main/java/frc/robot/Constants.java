// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.Turret;


/** Add your docs here. */
public final class Constants {

  //Motor Controller Configs for different Amp Allowances
  public static final SparkMaxConfig kTwntyAmp = new SparkMaxConfig();
  public static final SparkMaxConfig kFortyAmp = new SparkMaxConfig();
  public static final SparkMaxConfig kThirtyAmp = new SparkMaxConfig();

  public static final SparkMaxConfig kTurFollow = new SparkMaxConfig();

  
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

    public static class LimelightConstants{
      //Red Hub Apriltag ID's (will need to be changed later)
      public static final int kRedHubFTMid = 1;
      public static final int kRedHubRTMid = 2;
      public static final int kRedHubLTMid = 3;
      //Blue Hub Apriltag ID's (will need to be changed later)
      public static final int kBlueHubFTMid = 4;
      public static final int kBlueHubRTMid = 5;
      public static final int kBlueHubLTMid = 6;

      public static NetworkTableEntry tX = Robot.limelightTable.getEntry("tx");
      public static NetworkTableEntry tY = Robot.limelightTable.getEntry("ty");
      public static NetworkTableEntry tA = Robot.limelightTable.getEntry("ta");

      public static double kTX = tX.getDouble(0);
    }

    public static class TurretConstants{
      public static double kHoodHome = 0;
      public static double kHoodMax = 100; //number will need to be adjusted later
      public static double kHoodPass = 50; //number will need to be adjusted later
      public static double kTurRotHome = 0;
      public static double kTurLTMax = 100; //number will need to be changed later
      public static double kTurRTMax = Turret.kTurRot.getEncoder().getPosition(); //number will need to be changed later

      public static final SparkMaxConfig kShootLead = new SparkMaxConfig();
      public static final SparkMaxConfig kShootFollow = new SparkMaxConfig();

      static{
        kShootLead
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(45);

        kShootFollow
          .apply(kShootLead)
          .follow(14, true);
      }
    }

    static{
      //Forty Amp Limit
      kFortyAmp.smartCurrentLimit(45);
      //Thirty Amp Limit
      kThirtyAmp.smartCurrentLimit(35);
      //Twenty Amp Limit
      kTwntyAmp.smartCurrentLimit(25);
      //Following motor

    
    }
}

