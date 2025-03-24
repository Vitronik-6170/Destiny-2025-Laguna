// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class SwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final CANcoderConfiguration m_coderConfig;
  private final CANcoder m_absEcoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int encoderId, double encoderOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_coderConfig = new CANcoderConfiguration();
    m_coderConfig.MagnetSensor = new MagnetSensorConfigs();
    m_coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_coderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    m_coderConfig.MagnetSensor.MagnetOffset = encoderOffset;

    m_absEcoder = new CANcoder(encoderId);
    m_absEcoder.getConfigurator().apply(m_coderConfig);

    double encoderRadians = m_absEcoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    m_chassisAngularOffset = getRelativeEncoderTurningPosition()-encoderRadians;
    //m_chassisAngularOffset = 0;

    m_desiredState.angle = new Rotation2d(getRelativeEncoderTurningPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(getRelativeEncoderTurningPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getRelativeEncoderTurningPosition() - m_chassisAngularOffset));
    
  }
  public double getEncoder(){
    return getRelativeEncoderTurningPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(getRelativeEncoderTurningPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians()*2.5, ControlType.kPosition);
    //SmartDashboard.putNumber("desired angle", correctedDesiredState.angle.getRadians());
  

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
  public void desiredDistance(double distance){
    double ticks = 6.8833, diameter = 0.1016;
    m_drivingClosedLoopController.setReference((distance*ticks)/(Math.PI*diameter), ControlType.kPosition);
  }

  public void resetToAbsolute(double ticks){
    double offset = m_absEcoder.getAbsolutePosition().getValueAsDouble()*ticks;
    m_turningClosedLoopController.setReference(offset, ControlType.kPosition);
    m_turningEncoder.setPosition(0);
  }


  public double getRelativeEncoderTurningPosition() {
    double valorNormalizado = m_turningEncoder.getPosition() % 21;
    return (valorNormalizado*2*Math.PI)/21;
  }
  public void setCoast(){
    Configs.MAXSwerveModule.drivingConfig.idleMode(IdleMode.kCoast);
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Configs.MAXSwerveModule.turningConfig.idleMode(IdleMode.kCoast);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setBrake(){
    Configs.MAXSwerveModule.drivingConfig.idleMode(IdleMode.kBrake);
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Configs.MAXSwerveModule.turningConfig.idleMode(IdleMode.kBrake);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
