// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class ElevatorChasis extends SubsystemBase {
  private final CANSparkMax MotorelevadorL = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax MotorelevadorR = new CANSparkMax(8, MotorType.kBrushless); 
  //Encoders  
  private final RelativeEncoder RightEncoder = MotorelevadorR.getEncoder();
  private final RelativeEncoder LeftEncoder = MotorelevadorL.getEncoder();



  public ElevatorChasis() {
    RightEncoder.setPosition(0);  RightEncoder.setPositionConversionFactor(Math.PI*(1/10));
    LeftEncoder.setPosition(0);   LeftEncoder.setPositionConversionFactor(Math.PI*(1/10));
    //Motores 
    MotorelevadorL.setInverted(true);
    MotorelevadorL.follow(MotorelevadorR); MotorelevadorL.set(0);MotorelevadorR.set(0);
    MotorelevadorL.setIdleMode(CANSparkMax.IdleMode.kBrake); MotorelevadorR.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("EncoderR", -RightEncoder.getPosition());
    //SmartDashboard.putNumber("EncoderL", LeftEncoder.getPosition());
  
  
   
  }
  public void SetMotorsElevador (double elevatorspeed)
   {
    MotorelevadorR.set(elevatorspeed);
   }
   public double getElevatorEncoder(){ return -RightEncoder.getPosition();}
  
}
