// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestMotor;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveMotor;
import frc.robot.commands.StopMotor;
import swervelib.SwerveInputStream;

//TO-DO turn needs to be inverted
public class RobotContainer {
  //creating an insatnce of subsystems in RobotContainer
  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final TestMotor testMotor = new TestMotor();

  //creating a controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    NamedCommands.registerCommand("example", Commands.print("Hello World"));

  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(), 
                                            () -> driverController.getLeftY(),
                                            () -> driverController.getLeftX())
                                            .withControllerRotationAxis(driverController::getRightX)
                                            .deadband(OperatorConstants.Deadzone)
                                            .scaleTranslation(1)
                                            .allianceRelativeControl(false);

    Command driveFieldOrientatedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {

    //holding X will make robot go slower
    driverController.x()
    .onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(MathUtil.interpolate(1.0, 0.1, 1));
    }))
    .onFalse(Commands.runOnce(() -> {
      driveAngularVelocity.scaleTranslation(1);
    }));

    //creating button press/release for commands
    driverController.povUp().whileTrue(new MoveMotor(testMotor));
    driverController.povUp().whileFalse(new StopMotor(testMotor));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}