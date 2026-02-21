// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  /*
   * this file will hold all code for shooting, rotation, hood flick, and general tower stuff
   * 
   * Check limelight docs for tracking tX and converting that to motor input via PID:
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
   * We have done similar things in code before, so also check preious years code on github.
   * 
   * Need a way to control hood angle based on distance from the apriltag with PID, how to do this can
   * also be found in the limelight docs.
   * 
   * Turret cannot rotate past 360 degrees when tracking. When it hits 359, it needs to rotate
   * back around the other direction with PID.
   * !!Might end up being about 180 degrees for TC depending on robot design. Final goal is 359
   * 
   * Should keep track of ID's based on alliance color, and field location.
   * 
   * Ability to track if motor is stalled or not, if stalled, run motor in reverse to clear
   * shooter jam.
   * 
   * Shooter motors should be able to spin and stop spinning.
   * 
   * Two Motors control the shooting of the ball, one NEO 550 controls the pivot of hood - Complete.
   * one motor controls the rotation of the turret, one motor for the tower - Complete.
   */
  /** Creates a new Turret. */

  //All Motors for turret ID's and Motor Type's assigned
  public SparkMax kLTShoot = new SparkMax(14, DriveConstants.NEO);
  public SparkMax kRTShoot = new SparkMax(15, DriveConstants.NEO);
  private SparkMax kTurRot = new SparkMax(16, DriveConstants.NEO);
  private SparkMax kHoodFlap = new SparkMax(17, DriveConstants.NEO550);
  

  //Tower Motor ID and Motor Type assignment
  private SparkMax kTowerMotor = new SparkMax(18, DriveConstants.NEO);

  private NetworkTable kLimeTable = NetworkTableInstance.getDefault().getTable("limelight");


  public Turret() {
    //Shooting Wheels configurations
    kLTShoot.configure(TurretConstants.kShootLead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kRTShoot.configure(TurretConstants.kShootFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    kTurRot.configure(Constants.kThirtyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kHoodFlap.configure(Constants.kTwntyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Configuration for Tower Motor
    kTowerMotor.configure(Constants.kFortyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void TurShoot(double Voltage){
    kLTShoot.setVoltage(Voltage);
  }
  

}
