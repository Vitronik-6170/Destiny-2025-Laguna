// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_Swerve;
  private final Arm m_Arm;
  private final Lift m_Lift;
  private final Intake m_Intake;

  private final Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCoral(SwerveDrive m_Swerve, Arm m_Arm ,Lift m_Lift, Intake m_Intake) {
    this.m_Swerve = m_Swerve;
    this.m_Arm = m_Arm;
    this.m_Lift = m_Lift;
    this.m_Intake = m_Intake;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Swerve, m_Arm, m_Lift, m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_Swerve.resetDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()<1){
      //m_Swerve.noPTR();
      m_Swerve.resetDrive();
      m_Swerve.driveOdometri(0);
    }else if (timer.get() > 1 && timer.get() < 2) {
      m_Lift.liftOut();
    }else if(timer.get() > 2.1 && timer.get() < 4.1){
      m_Arm.setToReef_L1();
    }else if(timer.get() > 4.2 && timer.get() < 6.5 ){
      m_Lift.goToFloor();
    }else if(timer.get() > 6.6 && timer.get() < 12 ){
      m_Swerve.driveOdometri(-2);
    }else if (timer.get() > 12.1 && timer.get() < 12.2){
      m_Swerve.resetDrive();
      m_Swerve.driveOdometri(0);
    }else if(timer.get() > 12.3 && timer.get() < 14){
      m_Intake.grabGamePiece(-0.1);
    }else{
      m_Swerve.drive(0, 0, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
