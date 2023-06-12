// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chasis;

public class Visioncmmd extends CommandBase {
  /** Creates a new Visioncmmd. */
  private final Chasis Chasiscontrol;
  private Double tx, ta;
  double velocidadL,velocidadR;
  public Visioncmmd(Chasis Chasiscontrol) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chasiscontrol);
    this.Chasiscontrol=Chasiscontrol;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    updateValues();
    
    if(ta!=0)
    {
      velocidadL=-(ta-3)/5;
      velocidadR=(ta-3)/5;
      
      if(Math.abs(tx)>=1)
      {
        velocidadL =velocidadL+((tx-0)/100*1.5);
        velocidadR =velocidadR+((tx-0)/100*1.5);
      }

    }
    else 
    {
      velocidadL=0;
      velocidadR=0;
    }
    Chasiscontrol.SetMotors(velocidadL, velocidadR);
  }

  // Called once the command ends or is interrupted.   
  private void updateValues(){
    this.tx = NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tx").getDouble(0);
    this.ta = NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(0);
  }
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
