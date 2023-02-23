// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunAutoCommand extends CommandBase {

  private final Supplier<Command> commandSupplier;
  private Command currentCommand;

  public RunAutoCommand(Supplier<Command> commandSupplier) {
    this.commandSupplier = commandSupplier;
  }

  @Override
  public void initialize() {
    currentCommand = commandSupplier.get();
    currentCommand.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    currentCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
