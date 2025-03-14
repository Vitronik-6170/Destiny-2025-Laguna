// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMaxConfig left_liftConfig;
  private final SparkMax left_liftMotor;
  private final RelativeEncoder left_liftEncoder;
  private final SparkClosedLoopController left_liftController;

  private final SparkMaxConfig right_liftConfig;
  private final SparkMax right_liftMotor;
  private final RelativeEncoder right_liftEncoder;
  private final SparkClosedLoopController right_liftController;

  private int levelIndex = 0;
  private static final double[] LEFT_LEVELS = {Constants.LiftConstants.kLEFTLiftReef_L1, Constants.LiftConstants.kLEFTLiftReef_L2, Constants.LiftConstants.kLEFTLiftReef_L3};
  private static final double[] RIGHT_LEVELS = {Constants.LiftConstants.kRIGHTLiftReef_L1, Constants.LiftConstants.kRIGHTLiftReef_L2, Constants.LiftConstants.kRIGHTLiftReef_L3};

  public Lift() {
    left_liftConfig = new SparkMaxConfig();
    left_liftConfig.idleMode(IdleMode.kBrake);
    left_liftConfig.inverted(false);
    left_liftConfig.closedLoop.pid(1, 0, 0);
    left_liftConfig.closedLoop.iZone(0.0002);
    left_liftConfig.closedLoop.outputRange(-Constants.LiftConstants.kLiftPower, Constants.LiftConstants.kLiftPower);

    right_liftConfig = new SparkMaxConfig();
    right_liftConfig.idleMode(IdleMode.kBrake);
    right_liftConfig.inverted(true);
    right_liftConfig.closedLoop.pid(1, 0, 0);
    right_liftConfig.closedLoop.iZone(0.0002);
    right_liftConfig.closedLoop.outputRange(-Constants.LiftConstants.kLiftPower, Constants.LiftConstants.kLiftPower);

    left_liftMotor = new SparkMax(Constants.LiftConstants.kLeftLiftMotorID, MotorType.kBrushless);
    left_liftEncoder = left_liftMotor.getEncoder();
    left_liftEncoder.setPosition(0);
    left_liftController = left_liftMotor.getClosedLoopController();
    left_liftMotor.configure(left_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    right_liftMotor = new SparkMax(Constants.LiftConstants.kRightLiftMotorID, MotorType.kBrushless);
    right_liftEncoder = right_liftMotor.getEncoder();
    right_liftEncoder.setPosition(0);
    right_liftController = right_liftMotor.getClosedLoopController();
    right_liftMotor.configure(right_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void increaseLevel() {
    if (levelIndex < LEFT_LEVELS.length - 1) {
      levelIndex++;
    }
    setLevel();
  }

  public void decreaseLevel() {
    if (levelIndex > 0) {
      levelIndex--;
    }
    setLevel();
  }

  private void setLevel() {
    double leftLevel = LEFT_LEVELS[levelIndex];
    double rightLevel = RIGHT_LEVELS[levelIndex];
    left_liftController.setReference(leftLevel, ControlType.kPosition);
    right_liftController.setReference(rightLevel, ControlType.kPosition);
  }
  public void setLevel(double targetLevel_R, double targetLevel_L) {
    right_liftController.setReference(targetLevel_R, ControlType.kPosition);
    left_liftController.setReference(targetLevel_L, ControlType.kPosition);
  }

  public void goToFloor() {
    double level = Constants.LiftConstants.kLiftFloor;
    left_liftController.setReference(level, ControlType.kPosition);
    right_liftController.setReference(level, ControlType.kPosition);
  }
  public void goToHuman() {
    double level = Constants.LiftConstants.kLiftFloor;
    left_liftController.setReference(level, ControlType.kPosition);
    right_liftController.setReference(level, ControlType.kPosition);
  }


  public void goToReef_L1() {
    double leftLevel = Constants.LiftConstants.kLEFTLiftReef_L2;
    double rightLevel = Constants.LiftConstants.kRIGHTLiftReef_L2;
    left_liftController.setReference(leftLevel, ControlType.kPosition);
    right_liftController.setReference(rightLevel, ControlType.kPosition);
  }
  public void goToReef_L2() {
    double leftLevel = Constants.LiftConstants.kLEFTLiftReef_L2;
    double rightLevel = Constants.LiftConstants.kRIGHTLiftReef_L2;
    left_liftController.setReference(leftLevel, ControlType.kPosition);
    right_liftController.setReference(rightLevel, ControlType.kPosition);
  }
  public void goToReef_L3() {
    double leftLevel = Constants.LiftConstants.kLEFTLiftReef_L3;
    double rightLevel = Constants.LiftConstants.kRIGHTLiftReef_L3;
    left_liftController.setReference(leftLevel, ControlType.kPosition);
    right_liftController.setReference(rightLevel, ControlType.kPosition);
  }
  public void liftOut(){
    double leftLevel = Constants.LiftConstants.kLEFTLiftTop;
    double rightLevel = Constants.LiftConstants.kRIGHTLiftTop;
    left_liftController.setReference(leftLevel, ControlType.kPosition);
    right_liftController.setReference(rightLevel, ControlType.kPosition);
  }

  public double getAbsoluteEncoderR()  {
    return right_liftEncoder.getPosition();
  }

  public double getAbsoluteEncoderL(){
    return left_liftEncoder.getPosition();
  }
  public void adjustLifterUp(){
    left_liftController.setReference(getAbsoluteEncoderL()+1, ControlType.kPosition);
    right_liftController.setReference(getAbsoluteEncoderR()+1, ControlType.kPosition);
  }
  public void adjustLifterDown(){
    left_liftController.setReference(getAbsoluteEncoderL()-1, ControlType.kPosition);
    right_liftController.setReference(getAbsoluteEncoderR()-1, ControlType.kPosition);
  }

  public void stop(){
    left_liftController.setReference(getAbsoluteEncoderL(), ControlType.kPosition);
    right_liftController.setReference(getAbsoluteEncoderR(), ControlType.kPosition);
  }
  public void setCoast(){
    left_liftConfig.idleMode(IdleMode.kCoast);
    left_liftMotor.configure(left_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_liftConfig.idleMode(IdleMode.kCoast);
    right_liftMotor.configure(right_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setBrake(){
    left_liftConfig.idleMode(IdleMode.kBrake);
    left_liftMotor.configure(left_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_liftConfig.idleMode(IdleMode.kBrake);
    right_liftMotor.configure(right_liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
