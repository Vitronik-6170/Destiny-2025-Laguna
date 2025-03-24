// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})
  private final Lift m_Lift;
  private final double targetLevel_R;
  private final double targetLevel_L;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(Lift m_Lift, double targetLevel_R, double targetLevel_L) {
    this.m_Lift = m_Lift;
    this.targetLevel_R = targetLevel_R;
    this.targetLevel_L = targetLevel_L;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Lift.setLevel(targetLevel_R, targetLevel_L);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_Lift.stop();      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Lift.getRelativeEncoderL()-targetLevel_L) < 1 || 
            Math.abs(m_Lift.getRelativeEncoderR()-targetLevel_R) < 1;
  }
}
