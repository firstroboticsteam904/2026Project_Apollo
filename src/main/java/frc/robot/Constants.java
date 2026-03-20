// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

  //Motor Controller Configs for different Amp Allowances
  public static final SparkMaxConfig ClimbPivotConfig = new SparkMaxConfig();
  public static final SparkMaxConfig ClimbExtendConfig = new SparkMaxConfig();

  public static final SparkMaxConfig LeftIntakePivotConfig = new SparkMaxConfig();
  public static final SparkMaxConfig RightIntakePivotConfig = new SparkMaxConfig();
  public static final SparkMaxConfig LeftIntakeRollerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig RightIntakeRollerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig BellyBeltConfig = new SparkMaxConfig();

  public static final SparkMaxConfig HoodConfig = new SparkMaxConfig();
  public static final SparkMaxConfig TurretConfig = new SparkMaxConfig();
  public static final SparkMaxConfig TowerConfig = new SparkMaxConfig();
  
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

    public static final class ShootConstants{
      public static final double HoodHome = 0;
      public static final double HoodShoot = 10;
      public static final double HoodPass = 25;
    }

    public static final class IntakeConstants{
      public static final double IntakeHome = 0;
      public static final double LeftIntakeExtend = 10;
      public static final double RightIntakeExtend = -10;
    }

    public static final class ClimbConstants{
      public static final double ClimbExtendHome = 0;
      public static final double ClimbExtendOut = 10;
      public static final double ClimbPivotHome = 0;
      public static final double ClimbPivotOut = 10;
    }

    static{
      ClimbPivotConfig.smartCurrentLimit(50).inverted(false).idleMode(IdleMode.kBrake);
      ClimbExtendConfig.smartCurrentLimit(50).inverted(false).idleMode(null);

      LeftIntakePivotConfig.smartCurrentLimit(50).inverted(false).idleMode(null);
      LeftIntakeRollerConfig.smartCurrentLimit(50).inverted(false).idleMode(null);
      RightIntakePivotConfig.smartCurrentLimit(50).inverted(false).idleMode(null);
      RightIntakeRollerConfig.smartCurrentLimit(50).inverted(false).idleMode(null);
      BellyBeltConfig.smartCurrentLimit(50).inverted(false).idleMode(null);

      HoodConfig.smartCurrentLimit(50).inverted(false).idleMode(IdleMode.kBrake);
      TurretConfig.smartCurrentLimit(50).inverted(false).idleMode(null);
      TowerConfig.smartCurrentLimit(50).inverted(false).idleMode(IdleMode.kBrake);
    
    }
}

