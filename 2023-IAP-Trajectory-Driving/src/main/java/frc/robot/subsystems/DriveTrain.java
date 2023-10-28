// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice; //Lines 9-19 could be importing files for the drive train's code
import com.ctre.phoenix.motorcontrol.NeutralMode;    //here to
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;   //here
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase //createing a class (public)
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //obtaining the shuffle board's tabs (boolean)
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry(); //the voltage of the right drive tain
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();

  /** Creates a new DriveTrain 
   * @param isInverted */
  public DriveTrain(boolean isInverted) 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //one part of the drive train
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //both set in a neutral mode (not moving)
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(true); //turing on the inverted for the left drive talon 
    rightDriveTalon.setInverted(false); //turing off the right driver talon


    leftDriveTalon.setSensorPhase(true); //turns on the left sensors 
    rightDriveTalon.setSensorPhase(true); //turns on the right sensors

    leftDriveTalon.configFactoryDefault(); //setting left drive talon to factory settings to run at optimal speeds
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();  //setting right drive train to facotry settings to run at optimal speeds 
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  private boolean extracted() {
    boolean isInverted;
    return (false);

  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);  //setting the speeds of the the drive talon 
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10); //resetting the sensors so it will
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  } //obtaining information from left & right drive talon and sensor

  public double ticksToMeters() {
    return (0.1524 * Math.PI / 4096) * getTicks();
  }

 
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() { //sends infomration to shufflebord
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle()); //getting motor's right and left voltages 
    SmartDashboard.putNumber("Right Ticks", rightDriveTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Ticks", leftDriveTalon.getSelectedSensorPosition()); //getting ticks (getting funtion)

    SmartDashboard.putNumber("Ticks to Metters", ticksToMeters());

    

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent()); //sets data in smart dashbord
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());    
  }
}