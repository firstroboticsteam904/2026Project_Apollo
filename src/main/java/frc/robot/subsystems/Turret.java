// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

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
  TalonFXS kLTShoot = new TalonFXS(14);
  TalonFXS kRTShoot = new TalonFXS(15);
  SparkMax kTurRot = new SparkMax(16, DriveConstants.NEO);
  SparkMax kHoodFlap = new SparkMax(17, DriveConstants.NEO550);
  
  //Tower Motor ID and Motor Type assignment
  SparkMax kTowerMotor = new SparkMax(18, DriveConstants.NEO);

  TalonFXSConfiguration MasterTalonConfig;

  public Turret() {
    MasterTalonConfig = new TalonFXSConfiguration();

    MasterTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //Configurations for Turret Motors
    kTurRot.configure(Constants.kThirtyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kHoodFlap.configure(Constants.kTwntyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Configuration for Tower Motor
    kTowerMotor.configure(Constants.kFortyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double HoodTicks(){
    double HoodTicks;
    HoodTicks = kHoodFlap.getEncoder().getPosition();
    SmartDashboard.putNumber("HoodTicks", HoodTicks);
    return HoodTicks;
  }

  public void HoodPower(double HoodVolts){
    kHoodFlap.setVoltage(HoodVolts);
  }

  public void ShootBall(double ShootVolts){
    kLTShoot.setVoltage(ShootVolts);
    kRTShoot.setVoltage(ShootVolts * -1);
  }

}
