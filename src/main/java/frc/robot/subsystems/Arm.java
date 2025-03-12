// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SparkMaxConfig armConfig;
  private final SparkMax armMotor;
  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  int noCoast=0;

  private int positionIndex = 0;
  private static final double[] POSITIONS = {Constants.ArmConstants.kArmReef_L1, Constants.ArmConstants.kArmReef_L2, Constants.ArmConstants.kArmReef_L3};

  public Arm() {
    armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake);
    armConfig.inverted(true);
    armConfig.absoluteEncoder.inverted(false);
    armConfig.absoluteEncoder.positionConversionFactor(2*Math.PI);
    armConfig.absoluteEncoder.zeroOffset(Constants.ArmConstants.kArmEncoderOffset);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armConfig.closedLoop.pid(1, 0, 0);
    armConfig.closedLoop.outputRange(-Constants.ArmConstants.kArmPower, Constants.ArmConstants.kArmPower);
    armConfig.closedLoopRampRate(0.001);

    armMotor = new SparkMax(Constants.ArmConstants.kArmMotorID, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder();
    armController = armMotor.getClosedLoopController();
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void increasePosition(){
    if (positionIndex < POSITIONS.length - 1) {
      positionIndex++;
    }
    setPosition();
  }

  public void decreasePosition(){
    if (positionIndex > 0) {
      positionIndex--;
    }
    setPosition();
  }

  public void adjustArmUp(){
    double position = getArmPosition();
    armController.setReference(position+0.349066, ControlType.kPosition);
  }
  public void adjustArmDown(){
    double position = getArmPosition();
    armController.setReference(position-0.349066, ControlType.kPosition);
  }

  private void setPosition() {
    double position = POSITIONS[positionIndex];
    armController.setReference(position, ControlType.kPosition);
  }

  public void grabFromFloor(){
    double position = Constants.ArmConstants.kArmFloor;
    armController.setReference(position, ControlType.kPosition);
  }

  public void grabFromHuman(){
    double position = Constants.ArmConstants.kArmHuman;
    armController.setReference(position, ControlType.kPosition);
  }

  public void setToReef_L1(){
    double position = Constants.ArmConstants.kArmReef_L1;
    armController.setReference(position, ControlType.kPosition);
  }

  public void setToReef_L2(){
    double position = Constants.ArmConstants.kArmReef_L2;
    armController.setReference(position, ControlType.kPosition);
  }

  public void setToReef_L3(){
    double position = Constants.ArmConstants.kArmReef_L3;
    armController.setReference(position, ControlType.kPosition);
  }

  public void armOut(){
    armController.setReference(Constants.ArmConstants.kArmOut, ControlType.kPosition);
  }
  public void stop(){
    armController.setReference(getArmPosition(), ControlType.kPosition);
  }
  public double getArmPosition(){
    return armEncoder.getPosition();
  }
  public void aceleration(double acc){
    armConfig.closedLoop.outputRange(-acc, acc);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setBrake(){
    armConfig.idleMode(IdleMode.kBrake);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setCoast(){
    if(noCoast==0){
      armConfig.idleMode(IdleMode.kCoast);
      armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      noCoast++;
    }else{
      armConfig.idleMode(IdleMode.kBrake);
      armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
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
