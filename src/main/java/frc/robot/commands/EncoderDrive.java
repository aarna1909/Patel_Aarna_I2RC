// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class EncoderDrive extends Command {
  /** Creates a new EncoderDrive. */
  private Drivetrain drivetrain;
  private double setpoint;
  public EncoderDrive(Drivetrain drivetrain, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    drivetrain.tankDrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(0.8, 0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getMeters() >= setpoint){
      return true;
    }
    else return false;
  }
}