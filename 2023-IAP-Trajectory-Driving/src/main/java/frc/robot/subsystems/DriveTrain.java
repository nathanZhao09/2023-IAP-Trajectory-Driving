// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice; //Lines 9-19 could be importing files for the drive train's code
import com.ctre.phoenix.motorcontrol.NeutralMode;    //here to
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;   //here
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase //createing a class (public)
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //obtaining the shuffle board's tabs (boolean)
  private double simLeftVoltage; //the voltage of the right drive tain
  private double simRightVoltage;
  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_Field2d;
  private DifferentialDrive drive;

 
  /** Creates a new DriveTrain 
   * @param isInverted */
  public DriveTrain(boolean isInverted) 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //one part of the drive train
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    leftDriveSim = leftDriveTalon.getSimCollection();
    rightDriveSim = rightDriveTalon.getSimCollection();
    drive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);
  
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
    m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112,                  // The track width is 0.7112 meters.
    
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
      m_Field2d = new Field2d();
      m_Field2d.setRobotPose(new Pose2d(0,0,new Rotation2d()));
      

  }

  private boolean extracted() {
    boolean isInverted;
    return (false);

  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);  //setting the speeds of the the drive talon 
    leftDriveTalon.set(leftSpeed);
    simLeftVoltage = leftSpeed*12.0;
    simRightVoltage= rightSpeed*12.0;
    drive.feed();
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

    

    
  }
  @Override
  public void simulationPeriodic(){
    m_driveSim.setInputs(simLeftVoltage, simRightVoltage);
    m_driveSim.update(0.02);
    m_Field2d.setRobotPose(m_driveSim.getPose());
    SmartDashboard.putData("Feild", m_Field2d);
  }
}

//Create our feedforward gain constants (from the identification
// tool)
