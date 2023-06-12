// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chasis;
import edu.wpi.first.wpilibj.Timer;

public class Autonomo extends CommandBase {
 private final Chasis Chasiscontrol;
 private final int Setpoint;
 private double kp=0.02,ki=0.02;
 private double EncoderR=0,EncoderL=0;
 private double errorPR,errorPL,errorIL,errorIR,ultimotiempo ;
 private double ZonaIntegral;
 private double kd=0.002;
 private double errorDL,errorDR;
 private double LastErrorR = 0, LastErrorL = 0;


  public Autonomo(Chasis Chasiscontrol, int Setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chasiscontrol);
    this.Chasiscontrol=Chasiscontrol;
    this.Setpoint=Setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EncoderL=0;
    EncoderR=0;
    Chasiscontrol.resetEncode();
    ZonaIntegral= Math.abs((Setpoint-Chasiscontrol.getLeftEncoder())*.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocidadL,velocidadR;
    
    EncoderL=Chasiscontrol.getLeftEncoder();
    EncoderR=Chasiscontrol.getRightEncoder();
    errorPL=Setpoint-EncoderL;
    errorPR=Setpoint-EncoderR;
    //I
    double dt =Timer.getFPGATimestamp()-ultimotiempo;
    if(Math.abs(errorPL)<ZonaIntegral)
    {
      errorIL+=errorPL*dt;
    }
    if(Math.abs(errorPR)<ZonaIntegral)
    {errorIR +=errorPR*dt;}
    //D
    if(Math.abs(errorPL)<ZonaIntegral)
    {errorDL=(errorPL-LastErrorL)/dt;}
    if(Math.abs(errorPR)<ZonaIntegral)
    {errorDR=(errorPR-LastErrorL)/dt;}

      velocidadL=kp*errorPL+ki*errorIL+kd*errorDL;
      velocidadR=kp*errorPR+ki*errorIR+kd*errorDR;
   
    Chasiscontrol.SetMotors(velocidadL, -velocidadR);
    ultimotiempo=Timer.getFPGATimestamp();
    LastErrorL=errorPL;
    LastErrorR=errorPR;
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
