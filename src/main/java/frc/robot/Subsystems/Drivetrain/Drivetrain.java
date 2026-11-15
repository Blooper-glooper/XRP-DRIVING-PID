// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final XRPMotor leftMotor;
  private final XRPMotor rightMotor;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final XRPGyro gyro;

  private final PIDController distanceController;
  private final PIDController turnController;
  private final SimpleMotorFeedforward distanceFeedforward;

  private double lastDistance = 0;
  private boolean isDriving = false;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftMotor = new XRPMotor(DrivetrainConstants.leftMotorID);
    rightMotor = new XRPMotor(DrivetrainConstants.rightMotorID);

    leftEncoder = new Encoder(DrivetrainConstants.leftEncoderA, DrivetrainConstants.leftEncoderB);
    rightEncoder = new Encoder(DrivetrainConstants.rightEncoderA, DrivetrainConstants.rightEncoderB);

    gyro = new XRPGyro();

    distanceController = new PIDController(
      DrivetrainConstants.drive_kP, 
      DrivetrainConstants.drive_kI, 
      DrivetrainConstants.drive_kD);

    turnController = new PIDController(
      DrivetrainConstants.turn_kP, 
      DrivetrainConstants.turn_kI, 
      DrivetrainConstants.turn_kD);

    distanceFeedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);

    rightMotor.setInverted(DrivetrainConstants.isRightInverted);

    leftEncoder.setDistancePerPulse(DrivetrainConstants.distancePerPulse);
    rightEncoder.setDistancePerPulse(DrivetrainConstants.distancePerPulse);

    distanceController.setTolerance(DrivetrainConstants.driveTolerance);

    turnController.enableContinuousInput(0, 360);
  }

  public void driveDistance(double distanceMeters) {
    leftEncoder.reset();
    rightEncoder.reset();
    distanceController.reset();
    turnController.reset();

    distanceController.setSetpoint(distanceMeters);
    turnController.setSetpoint(gyro.getAngleZ());

    lastDistance = 0;
    isDriving = true;
  }

  @Override
  public void periodic() {

    double currentDistance = (leftEncoder.getDistance() + rightEncoder.getDistance())/2.0;

    double currentVelocity = (currentDistance - lastDistance) / 0.02;
    lastDistance = currentDistance;

    double distanceOutput = distanceController.calculate(currentDistance);
    double feedforwardOutput = distanceFeedforward.calculate(currentVelocity);
    
    double baseVoltage = distanceOutput + feedforwardOutput;

    double turnVoltage = turnController.calculate(gyro.getAngleZ());

    double leftVoltage = baseVoltage - turnVoltage;
    double rightVoltage = baseVoltage + turnVoltage;

    leftVoltage = MathUtil.clamp(leftVoltage, -5.0, 5.0);
    rightVoltage = MathUtil.clamp(rightVoltage, -5.0, 5.0);

    SmartDashboard.putNumber("CurrentDistance", currentDistance);
    SmartDashboard.putNumber("CurrentVelocity", currentVelocity);
    SmartDashboard.putNumber("LeftVoltage", leftVoltage);
    SmartDashboard.putNumber("RightVoltage", rightVoltage);
    SmartDashboard.putNumber("Error", distanceController.getError());
    SmartDashboard.putNumber("TurnError", turnController.getError());

    if(!isDriving) {
      return;
    }

    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);

    if(distanceController.atSetpoint()) {
      isDriving = false;
      leftMotor.setVoltage(0);
      rightMotor.setVoltage(0);
    }
  }

  public void stop() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
    isDriving = false;
  }

  public boolean isDriving() {
    return isDriving;
  }
}
