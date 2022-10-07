// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DriveByTrajectory extends CommandBase {
  SwerveDriveKinematics d;
  SwerveDriveKinematicsConstraint autoVoltageConstraint = new SwerveDriveKinematicsConstraint(d, 0);
  private final Drivetrain m_drivetrainSubsystem;
  
    
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                0,
                0)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(d
               // DriveConstants.kDriveKinematics
              )
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
  Trajectory exampleTrajectory;
  private long start;
  /** Creates a new DriveByTrajectory. */
  public DriveByTrajectory(Drivetrain drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = HALUtil.getFPGATime();
    exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State goal = exampleTrajectory.sample(HALUtil.getFPGATime() - start);
    
    m_drivetrainSubsystem.drive(m_drivetrainSubsystem.follower.calculate(
      m_drivetrainSubsystem.odometer.getPoseMeters(),
      goal,
      goal.poseMeters.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HALUtil.getFPGATime() - start >= exampleTrajectory.getStates().size();
  }
}
