// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  public DriveTrain dt;
  public Joystick joy;

  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain dt, Joystick j) {
    this.dt = dt;
    this.joy = j;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    dt.tankDrive(0.0, 0.0);
  }

  // Called every time the scheduler runs whie the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(10);

    double rightPowerRaw = joy.getRawAxis(10);

    dt.tankDrive(leftPowerRaw*-0.2, rightPowerRaw*-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.resetEncoders();
    dt.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
