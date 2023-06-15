// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chasis;
import edu.wpi.first.wpilibj.Timer;
public class Balancecmmd extends CommandBase {
  private final Chasis chasiscontrol;
  private final int Setpoint;
  private double kp=0.02,ki=0.02;
  private double EncoderR=0,EncoderL=0;
  private double errorPR,errorPL,errorIL,errorIR,ultimotiempo ;
  private double ZonaIntegral;
  private double kd=0.002;
  private double pitch;
  private double errorDL,errorDR;
  private double LastErrorR = 0, LastErrorL = 0;
  private double estado=0;
  private double velocidadL,velocidadR;
  /** Creates a new Balancecmmd. */
  public Balancecmmd(Chasis chasiscontrol, int Setpoint) {
    addRequirements(chasiscontrol);
    this.chasiscontrol=chasiscontrol;
    this.Setpoint=Setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EncoderL=0;
    EncoderR=0;
    chasiscontrol.resetEncode();
    ZonaIntegral= Math.abs((Setpoint-chasiscontrol.getLeftEncoder())*.1);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     /* 
    EncoderL=chasiscontrol.getLeftEncoder();
    EncoderR=chasiscontrol.getRightEncoder();
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
   
    chasiscontrol.SetMotors(velocidadL, -velocidadR);
    ultimotiempo=Timer.getFPGATimestamp();
    LastErrorL=errorPL;
    LastErrorR=errorPR;
    estado=1;
    */
    
    
    pitch=chasiscontrol.getPitch();
      chasiscontrol.SetMotors(pitch/-20, pitch/20);
     
     
     
    
     
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
