// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Intakecmmd extends CommandBase {
  private final Intake Intakeshooter;
  private final Supplier <Double> Sadstick;
  public Intakecmmd(Intake Intakeshooter, Supplier<Double> Sadstick) {
    addRequirements(Intakeshooter);
    this.Intakeshooter=Intakeshooter;
    this.Sadstick=Sadstick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double Joystick= Sadstick.get();
    double Velocidad=0;
    if(Math.abs(Joystick)<=.25) {Joystick=0.0;} 
    if(Joystick>.20)
    {
      Velocidad=Joystick*0.7;
    }
    if (Joystick<-.20)
    {
      Velocidad=-Joystick*0.7;
    }
    Intakeshooter.SetMotors(Velocidad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intakeshooter.SetMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
