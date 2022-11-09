// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbControl extends CommandBase {
  private final ClimbSubsystem climbSubsystem;
  private final Supplier<Double> climbArmSupplier;
  private final Supplier<Double> climbPoleSupplier;

  public ClimbControl(ClimbSubsystem climbSubsystem, Supplier<Double> climbArmSupplier, Supplier<Double> climbPoleSupplier) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.climbArmSupplier = climbArmSupplier;
    this.climbPoleSupplier = climbPoleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.holdPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.midClimb(climbArmSupplier.get());
    climbSubsystem.highArms(climbPoleSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
