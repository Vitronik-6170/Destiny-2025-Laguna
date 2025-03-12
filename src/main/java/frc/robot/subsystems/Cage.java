// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cage extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMaxConfig left_cageConfig;
  private final SparkMax left_cageMotor;

  private final SparkMaxConfig right_cageConfig;
  private final SparkMax right_cageMotor;
  private final AbsoluteEncoder right_cageAbsoluteEncoder;
  private final SparkClosedLoopController right_cageController;
  
  public Cage() {
    right_cageConfig = new SparkMaxConfig();
    right_cageConfig.idleMode(IdleMode.kBrake);
    right_cageConfig.inverted(true);
    right_cageConfig.absoluteEncoder.inverted(true );
    right_cageConfig.absoluteEncoder.zeroOffset(Constants.CageConstants.kCageEncoderOffset);
    right_cageConfig.absoluteEncoder.positionConversionFactor(2*Math.PI);
    right_cageConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    right_cageConfig.closedLoop.pid(1, 0.2, 0);
    right_cageConfig.closedLoop.outputRange(-Constants.CageConstants.kCagePower, Constants.CageConstants.kCagePower);

    left_cageConfig = new SparkMaxConfig();
    left_cageConfig.idleMode(IdleMode.kBrake);
    left_cageConfig.inverted(false);

    right_cageMotor = new SparkMax(Constants.CageConstants.kRightCageMotorID, MotorType.kBrushless);
    right_cageAbsoluteEncoder = right_cageMotor.getAbsoluteEncoder();
    right_cageController = right_cageMotor.getClosedLoopController();
    right_cageMotor.configure(right_cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    left_cageMotor = new SparkMax(Constants.CageConstants.kLeftCageMotorID, MotorType.kBrushless);
    left_cageConfig.follow(right_cageMotor,true);
    left_cageMotor.configure(left_cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  public void prepareToHang() {
    right_cageController.setReference(Constants.CageConstants.kCagePrepareToHang, ControlType.kPosition);
  }
  public void hang(){
    right_cageController.setReference(Constants.CageConstants.kCageHang, ControlType.kPosition);
  }
  public void hanging() {
    // if(right_cageAbsoluteEncoder.getPosition()>Constants.CageConstants.kCageInit){
    //   right_cageMotor.set(0);
    // }else{
    //   right_cageMotor.set(Constants.CageConstants.kRight_PowerCageHang);
    // }
    right_cageMotor.set(Constants.CageConstants.kRight_PowerCageHang);
  }
  public void desHanging() {
    right_cageMotor.set(-Constants.CageConstants.kRight_PowerCageHang);
  }
  public void initHang(){
    right_cageController.setReference(Constants.CageConstants.kCageInit, ControlType.kPosition);
  }
  public void stopCage() {
    right_cageMotor.set(0);
    left_cageMotor.set(0);
  }
  public double getAbsoluteEncoderCage(){
    return right_cageAbsoluteEncoder.getPosition();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
