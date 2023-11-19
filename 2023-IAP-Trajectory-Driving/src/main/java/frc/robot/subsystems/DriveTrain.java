// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final DifferentialDriveOdometry odometry;

 
  /** Creates a new DriveTrain 
   * @param isInverted */
  public DriveTrain()
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //one part of the drive train
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    leftDriveSim = leftDriveTalon.getSimCollection();
    rightDriveSim = rightDriveTalon.getSimCollection();
    drive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //both set in a neutral mode (not moving)
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false); //turing on the inverted for the left drive talon 
    rightDriveTalon.setInverted(true); //turing off the right driver talon


    leftDriveTalon.setSensorPhase(true); //turns on the left sensors 
    rightDriveTalon.setSensorPhase(true); //turns on the right sensors

    leftDriveTalon.configFactoryDefault(); //setting left drive talon to factory settings to run at optimal speeds
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();  //setting right drive train to facotry settings to run at optimal speeds 
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    
    // Create the simulation model of our drivetrain.
 m_driveSim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(Constants.RamseteConstants.kV,
      Constants.RamseteConstants.kA, Constants.RamseteConstants.kVangular,
      Constants.RamseteConstants.kAangular),
      DCMotor.getCIM(1), // 1 CIM motor on each side of the drivetrain.
      10.71, // 10.71:1 gearing reduction.
      Constants.RamseteConstants.kTrackwidthMeters, // The track width is 0.7112
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.                // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

int x = 4;
int y = 4;
VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

      m_driveSim.setPose(new Pose2d(0,0, new Rotation2d(0)));
      odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());
      m_Field2d = new Field2d();
      m_Field2d.setRobotPose(new Pose2d(0,0,new Rotation2d()));
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
    return -navx.getAngle(); 
  }
  public double metersToTicks(double positionMeters){
    return (positionMeters / (0.1524 * Math.PI)*4096);
  }
 
  public void resetNavx(){
    navx.reset();
  }
  public double getLeftDistance() {
    return leftDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }
  /**
   * Returns displacement of right side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getRightDistance() {
    return rightDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns linear velocity of left side of chassis.
   * 
   * @return the linear velocity in meters/s (m/s)
   */
  public double getLeftSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }
  public double getRightSpeed() {
    return (rightDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns linear velocity of right side of chassis.
   * @return 
   * 
   * @return the linear velocity in meters/s (m/s)
   */

  @Override
  public void periodic() { //sends infomration to shufflebord
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle()); //getting motor's right and left voltages 
    SmartDashboard.putNumber("Right Ticks", rightDriveTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Ticks", leftDriveTalon.getSelectedSensorPosition()); //getting ticks (getting funtion)

    SmartDashboard.putNumber("Ticks to Meters", ticksToMeters());
  }
    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition(
          navx.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
    }
  
    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      drive.arcadeDrive(fwd, rot);
    }
  
    /**
     * Controls the left and right sides of the drive directly with voltages.
     * Voltage is the native unit of Feedforward with WPILib
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      // drive.setSafetyEnabled(false);
      simLeftVoltage = leftVolts;
      simRightVoltage = rightVolts;
      leftDriveTalon.setVoltage(leftVolts);
      rightDriveTalon.setVoltage(rightVolts);
      // WPILib would spit out "Looptime Overrun!" if this isn't included!
      drive.feed();
    }
    

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  @Override
  public void simulationPeriodic(){
    m_driveSim.setInputs(simLeftVoltage, simRightVoltage);
    m_driveSim.update(0.02);
    m_Field2d.setRobotPose(m_driveSim.getPose());
    
    SmartDashboard.putData("Field", m_Field2d);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(m_driveSim.getHeading().getDegrees());
    leftDriveTalon.setSelectedSensorPosition(metersToTicks(m_driveSim.getLeftPositionMeters()),0,10);
    rightDriveTalon.setSelectedSensorPosition(metersToTicks(m_driveSim.getRightPositionMeters()),0,10);

    rightDriveSim.setQuadratureRawPosition( 
      distanceToNativeUnits(
            m_driveSim.getRightPositionMeters())); //Gets the position for the right motor in meters
    rightDriveSim.setQuadratureVelocity(
        velocityToNativeUnits(
            m_driveSim.getRightVelocityMetersPerSecond())); //Gets the velocity for the right motor in meters per second
       
  }
    private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters
        / (Math.PI * Units.inchesToMeters(Constants.DriveToLineConstants.wheelDiameterInInches));
    double motorRotations = wheelRotations * 1.0;
    int sensorCounts = (int) (motorRotations * 4096.0);
    return sensorCounts;
  }
  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / 4096.0;
    double wheelRotations = motorRotations / 1.0;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(6.0));
    return positionMeters;
  }
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }


  public int velocityToNativeUnits(double velocityMetersPerSecond) {
    // Previous mistake: multiply this by 2
    // Consequences: had to set the constant to 0.5 less
    // Now it works without the 2
    double wheelRotationsPerSecond = velocityMetersPerSecond
        / (Math.PI * Units.inchesToMeters(Constants.DriveToLineConstants.wheelDiameterInInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }
  public Field2d getField2d() {
    return m_Field2d;
    }
  
}

//Create our feedforward gain constants (from the identification
// tool)
