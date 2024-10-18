// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class PIDTurnCCW extends Command {
  private final Drivetrain drive;
  private final double setPointAngle;
  private final PIDController controller;

  /** Creates a new PID. */
  public PIDTurnCCW(Drivetrain drive, double wantedAngle, PIDController pidController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.setPointAngle = wantedAngle;
    this.controller = pidController;
    addRequirements(drive);
    controller.setTolerance(5);
  }

  private double chassisAngle() {
    return drive.getAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.reset();
    drive.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(chassisAngle(), setPointAngle);
    drive.tankDrive(output * -1, output * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}