// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AdjustArmDown;
import frc.robot.commands.AdjustArmUp;
import frc.robot.commands.AdjustLifterDown;
import frc.robot.commands.AdjustLifterUp;
import frc.robot.commands.DesHanging;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Floor;
import frc.robot.commands.Hang;
import frc.robot.commands.Hanging;
import frc.robot.commands.Human;
import frc.robot.commands.PrepareToHang;
import frc.robot.commands.ReefDown;
import frc.robot.commands.ReefUp;
import frc.robot.commands.ReleaseArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cage;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Intake m_Intake;
  public final Arm m_Arm;
  public final Lift m_Lift; 
  public final Wrist m_Wrist;
  public final Cage m_Cage;
  public final SwerveDrive m_SwerveDrive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public static CommandXboxController m_mechanismsController =
      new CommandXboxController(OperatorConstants.kMechanismsControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Intake = new Intake();
    m_Arm = new Arm();
    m_Lift = new Lift();
    m_Wrist = new Wrist();
    m_Cage = new Cage();
    m_SwerveDrive = new SwerveDrive();


    configureBindings();

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_mechanismsController.rightBumper().whileTrue(new ReefUp(m_Arm, m_Lift, m_Wrist));
    m_mechanismsController.leftBumper().whileTrue(new ReefDown(m_Arm, m_Lift, m_Wrist));
    m_mechanismsController.a().whileTrue(new Floor(m_Arm, m_Lift, m_Wrist));  
    m_mechanismsController.x().whileTrue(new Human(m_Arm, m_Lift, m_Wrist));
    m_mechanismsController.pov(0).whileTrue(new AdjustArmUp(m_Arm));
    m_mechanismsController.pov(180).whileTrue(new AdjustArmDown(m_Arm));
    //m_mechanismsController.pov(270).whileTrue(new AdjustLifterDown(m_Lift));
    //m_mechanismsController.pov(90).whileTrue(new AdjustLifterUp(m_Lift));
    m_mechanismsController.b().whileTrue(new Hanging(m_Cage));
    m_mechanismsController.y().whileTrue(new DesHanging(m_Cage));
    m_mechanismsController.leftStick().whileTrue(new ReleaseArm(m_Lift));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_SwerveDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_SwerveDrive::setModuleStates,
        m_SwerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_SwerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_SwerveDrive.drive(0, 0, 0, false));
  }
}
