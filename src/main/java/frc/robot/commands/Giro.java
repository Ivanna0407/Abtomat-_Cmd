// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chasis;
//import com.kauailabs.navx.frc.AHRS;
import frc.robot.NavX.AHRS;
import edu.wpi.first.wpilibj.SPI;
public class Giro extends CommandBase {
  /** Creates a new Giro. */
  private final Chasis Chasiscontrol;
    //Gyro 
    //AHRS gyroscopio = new AHRS(SPI.Port.kMXP);
   private double Yaw;
   

  public Giro(Chasis Chasiscontrol) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chasiscontrol);
    this.Chasiscontrol=Chasiscontrol;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Yaw=Chasiscontrol.getYaw();
    double velocidadL, velocidadR;
    if (Yaw>90)
    {
      velocidadL=.4;
      velocidadR=-.4;
    }
    else{
      velocidadL=0;
      velocidadR=0;
    }
    Chasiscontrol.SetMotors(velocidadL, velocidadR);
    
  
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
