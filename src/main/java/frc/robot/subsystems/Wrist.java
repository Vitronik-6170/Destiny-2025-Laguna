// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMaxConfig wristConfig;
  private final SparkMax wristMotor;
  private final AbsoluteEncoder wristEncoder;
  private final SparkClosedLoopController wristController;

  private int positionWrist = 0;
  private static final double [] POSITIONS = {Constants.WristConstants.kWristReeft_L1, Constants.WristConstants.kWristReeft_L2, Constants.WristConstants.kWristReeft_L2};

  public Wrist() {
    
    wristConfig = new SparkMaxConfig();
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.inverted(false);
    wristConfig.absoluteEncoder.inverted(false);
    wristConfig.absoluteEncoder.positionConversionFactor(2*Math.PI);
    wristConfig.absoluteEncoder.zeroOffset(Constants.WristConstants.kWristEncoderOffset);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.closedLoop.pid(1, 0, 0);
    wristConfig.closedLoop.outputRange(-Constants.WristConstants.kWristPower, Constants.WristConstants.kWristPower);
    wristConfig.closedLoop.positionWrappingEnabled(true);
    wristConfig.closedLoop.positionWrappingInputRange(0, 2*Math.PI);

    wristMotor = new SparkMax(Constants.WristConstants.kWristMotorID, MotorType.kBrushless);
    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristController = wristMotor.getClosedLoopController();
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void increasePosition(){
    if (positionWrist < POSITIONS.length - 1) {
      positionWrist++;
    }
    setPosition();
  }

  public void decreasePosition(){
    if (positionWrist > 0) {
      positionWrist--;
    }
    setPosition();
  }


  private void setPosition() {
    double position = POSITIONS[positionWrist];
    wristController.setReference(position, ControlType.kPosition);
  }

  public void verticalWrist() {
    wristController.setReference(Constants.WristConstants.verticalWrist, ControlType.kPosition);
  }
  public void horizontalWrist() {
    wristController.setReference(Constants.WristConstants.horizontalWrist, ControlType.kPosition);
  }
  public void floorWrist() {
    wristController.setReference(Constants.WristConstants.FloorWrist, ControlType.kPosition);
  }

  public void getAbsoluteEncoder() {
    wristEncoder.getPosition();
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
