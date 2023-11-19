// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.commands.PidTurnCCW;
import frc.robot.commands.Ramsete;
import frc.robot.commands.AutoDrive;


public class RobotContainer {
  Ramsete ramsete = new Ramsete();
  public static DriveTrain dt = new DriveTrain();
  Joystick j = new Joystick(0);
  public RobotContainer() {
    dt.setDefaultCommand(new TankDrive(dt, j));
    configureBindings();
  }


  private void configureBindings() {}




public Command getAutonomousCommand() {
return ramsete;
}
}
