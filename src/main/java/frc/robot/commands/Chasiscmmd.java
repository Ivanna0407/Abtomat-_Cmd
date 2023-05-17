// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.ChasisDriveSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Chasiscmmd extends CommandBase {
  private final Chasis Chasiscontrol;
  private final Supplier <Double> speed, giro, reversa;
  private final Supplier <Boolean> turbo;
  public Chasiscmmd( Chasis Chasiscontrol, Supplier<Double> speed, Supplier<Double> giro, Supplier<Double> reversa, Supplier<Boolean> turbo) {
    addRequirements(Chasiscontrol);
    this.Chasiscontrol=Chasiscontrol;
    this.speed=speed;
    this.giro=giro;
    this.reversa=reversa;
    this.turbo=turbo;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //aqui va el algoritmo suerte =D
    double velocidadap = speed.get()-reversa.get();
    double giroap =giro.get();
    if (Math.abs(giroap)<=.25){giroap=0;}
    double max, velocidadL, velocidadR;
    if(turbo.get()==true)
    {
      max=.4;
    }else{max=1;}
    if(Math.abs(velocidadap)>.15)
    {
      velocidadL =(velocidadap+giroap)*max;
      velocidadR =(velocidadap+giroap)*max;
    }else{velocidadL = 0; velocidadR = 0;}
    Chasiscontrol.SetMotors(velocidadL, velocidadR);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Chasiscontrol.SetMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
