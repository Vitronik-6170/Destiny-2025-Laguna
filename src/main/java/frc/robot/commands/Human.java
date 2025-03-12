// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Human extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_Arm;
  private final Lift m_Lift;
  private final Wrist m_Wrist;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Human(Arm m_Arm,Lift m_Lift, Wrist m_Wrist) {
    this.m_Arm = m_Arm;
    this.m_Lift = m_Lift;
    this.m_Wrist = m_Wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm,m_Lift, m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.grabFromHuman();
    m_Lift.goToHuman();
    m_Wrist.horizontalWrist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
