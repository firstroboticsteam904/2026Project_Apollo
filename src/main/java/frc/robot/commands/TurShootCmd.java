// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurShootCmd extends Command {
  /** Creates a new TurShootCmd. */
  private final Turret kTurret;
  public TurShootCmd(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    kTurret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
     * Voltage controls the speed of the motor. Use this over .set as Voltage will assign a voltage to be sent
     * over sending a percentage of the battery power currently avalible. Avoid setting at 12v as that is
     * the normal opereating voltage of the battery, and will dip as the match goes on.
     * 
     * This might be changed to run on velocity/rpm later, for now use voltage.
    */
    kTurret.TurShoot(10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
