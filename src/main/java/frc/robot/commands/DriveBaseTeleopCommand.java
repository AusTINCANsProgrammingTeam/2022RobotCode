// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;
import java.util.function.Supplier;

public class DriveBaseTeleopCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBaseSubsystem driveBaseSubsystem;

  private final Supplier<Double> speedSupplier;
  private final Supplier<Double> rotationSupplier;

  public DriveBaseTeleopCommand(
      DriveBaseSubsystem driveBaseSubsystem,
      Supplier<Double> speedSupplier,
      Supplier<Double> rotationSupplier) {
    addRequirements(driveBaseSubsystem);
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveBaseSubsystem.arcadeDrive(speedSupplier.get(), rotationSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stopDriveMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
