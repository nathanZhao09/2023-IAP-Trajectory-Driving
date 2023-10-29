// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  DriveTrain dt = new DriveTrain(false);
  Joystick joy1 = new Joystick(0);

  public RobotContainer() {
    //dt = new DriveTrain(true);
    dt.setDefaultCommand(new TankDrive(dt, joy1));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
