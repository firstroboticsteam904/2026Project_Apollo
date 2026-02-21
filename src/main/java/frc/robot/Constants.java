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
    
    //Maximum speed allowance for Drivetrain
    public static final double maximumSpeed = Units.feetToMeters(13.59);
    
    //Motor type shortcut for SparkMAX set up.
    public static final MotorType NEO = MotorType.kBrushless;
    public static final MotorType NEO550 = MotorType.kBrushless;

    static{
      //Forty Amp Limit
      kFortyAmp.smartCurrentLimit(45);
      //Thirty Amp Limit
      kThirtyAmp.smartCurrentLimit(35);
      //Twenty Amp Limit
      kTwntyAmp.smartCurrentLimit(25);

    
    }
}

