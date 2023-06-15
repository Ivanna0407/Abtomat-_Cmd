// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.kauailabs.navx.frc.AHRS;
import frc.robot.NavX.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class Chasis extends SubsystemBase {
  private final CANSparkMax MotorRightM= new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax MotorRightS= new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax MotorLeftM= new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax MotorLeftS= new CANSparkMax(4, MotorType.kBrushless);
  AHRS ahrs = new AHRS(SPI.Port.kMXP,(byte)66);
  //Encoders 
  public final RelativeEncoder EncoderR= MotorRightM.getEncoder();
  private final RelativeEncoder EncoderL= MotorLeftM.getEncoder();
  //Gyro 
  //AHRS gyroscopio = new AHRS(SPI.Port.kMXP);

  public Chasis() {
    //Se resetea variables y encoders
    EncoderR.setPosition(0);  EncoderR.setPositionConversionFactor(6*Math.PI*(1/10.71));
    EncoderL.setPosition(0);   EncoderL.setPositionConversionFactor(6*Math.PI*(1/10.71));
    //gyroscopio.zeroYaw();
    
    

    //Set motores en 0 y designamos motores esclavos e inversos
    MotorLeftS.follow(MotorLeftM);    MotorLeftM.set(0);
    MotorRightS.follow(MotorRightM);  MotorRightM.set(0); 

    //Configura todos los motores en Brake mode al recibir 0
    MotorLeftS.setIdleMode(CANSparkMax.IdleMode.kBrake);     MotorLeftM.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MotorRightS.setIdleMode(CANSparkMax.IdleMode.kBrake);    MotorRightM.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Derecho", -EncoderR.getPosition());
    SmartDashboard.putNumber("Encoder Izquierdo", EncoderL.getPosition());
    SmartDashboard.putNumber("Velocidad Motor Derecho", MotorRightM.get());
    SmartDashboard.putNumber("Velocidad Motor Izquierdo", MotorLeftM.get());
    //SmartDashboard.putNumber("Yaw",gyroscopio.getYaw() );
    SmartDashboard.putNumber("Giro",ahrs.getYaw() );
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
  }
  public void SetMotors(double LeftSpeed, double RightSpeed){
    MotorLeftM.set(LeftSpeed*.7); MotorRightM.set(RightSpeed*.7);
  }
  
  public double getRightEncoder(){ return -EncoderR.getPosition();}
  public double getLeftEncoder(){ return EncoderL.getPosition();}
    public double getYaw(){return ahrs.getYaw();}
   public double getPitch(){return ahrs.getPitch();}
  
  public void resetEncode(){
    EncoderR.setPosition(0); EncoderL.setPosition(0);
  }
}
