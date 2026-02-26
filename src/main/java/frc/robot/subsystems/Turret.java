// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
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
   * 
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
  public SparkMax kTurRot = new SparkMax(16, DriveConstants.NEO);
  private SparkMax kHoodFlap = new SparkMax(17, DriveConstants.NEO550);
  

  //Tower Motor ID and Motor Type assignment
  private SparkMax kTowerMotor = new SparkMax(18, DriveConstants.NEO);

  PIDController VisionPID = new PIDController(0, 0, 0);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html 
  // kS: minumum voltage to move motor. kV: Velocity Movement kA: acceleration
  SimpleMotorFeedforward TurFeedForward = new SimpleMotorFeedforward(0, 0, 0);

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

  public double kVisPID(){
    SmartDashboard.putNumber("Limelight TX", LimelightConstants.tX.getDouble(0));
    double kTurVisX = VisionPID.calculate(LimelightConstants.kTX, 0);
    SmartDashboard.putNumber("VisionPID", kTurVisX);

    return kTurVisX;
  }

  public void kVisReset(){
    VisionPID.reset();
  }
  
  //Speed to motors is set based on what is inputted into the voltage position in the TurShootCmd
  public void TurShoot(double Voltage){
    kLTShoot.setVoltage(Voltage);
  }

  public void TurRot(){
    double kTurFeed = TurFeedForward.calculate(1);
    kTurRot.setVoltage(kVisPID() + kTurFeed);
  }

  /*
   * Encoder ticks to Degrees
   * first, we must convert encoder ticks to rotations. To do that
   * we have to read the total encoder ticks coming from the motor.
   * Then we must find how many encoder ticks are given per revolution
   * of the motor shaft. On a 1:1 gear ratio, that would work fine.
   * This is however not a 1:1 ratio. We have a gear ratio of 53.2:1.
   * Meaning that for every 53.2 rotations from the motor, there is 1 full
   * rotation from the turret. Our formula will look something like this
   * 
   * TPR = (Total Encoder Tick / (42 * (53.2/1)))
   * 
   * Now to convert that to degrees of rotation we need to take that number and multiply
   * by 360.
   * 
   * TPR * 360
   */

   public double TurTicksToDegrees(){
      double TET = kTurRot.getEncoder().getPosition();
      double TPR = TET / (42 * (53.2/1));
      double Deg = TPR * 360;
      SmartDashboard.putNumber("Turret Position", Deg);

      return Deg;
   }



  public Command turRotCommand(){
    return this.startEnd(() -> {
      this.kTurRot.set(kVisPID());
      this.TurTicksToDegrees();
    }, () -> {
      this.kVisReset();
    });
  }

  public void Hoodflap(double Voltage){
    kHoodFlap.setVoltage(Voltage);
  }
  

}
