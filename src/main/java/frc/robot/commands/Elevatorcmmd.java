// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.ElevatorChasis;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Elevatorcmmd extends CommandBase {
  private final ElevatorChasis Elevator;
  private final Supplier <Double> Righttrigger,Lefttrigger;

  public Elevatorcmmd( ElevatorChasis Elevator , Supplier<Double> Righttrigger, Supplier<Double> Lefttriger){
    addRequirements(Elevator);
    this.Elevator = Elevator;
    this.Righttrigger = Righttrigger;
    this.Lefttrigger=Lefttriger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    if(Righttrigger.get() < 0.20 || Lefttrigger.get()<0.20)
    {
      speed=.07*(Righttrigger.get()-Lefttrigger.get());
    }

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
