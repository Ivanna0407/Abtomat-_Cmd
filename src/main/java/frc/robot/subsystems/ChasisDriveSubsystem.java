package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ChasisDriveSubsystem extends SubsystemBase {
  //Build de Motocontroladores (Motores)
  private final CANSparkMax MotorRightM = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax MotorRightS = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax MotorLeftM = new CANSparkMax(3, MotorType.kBrushless); 
  private final CANSparkMax MotorLeftS = new CANSparkMax(4, MotorType.kBrushless);

  //EncodersRevNeo
  private final RelativeEncoder RightEncoder = MotorRightM.getEncoder();
  private final RelativeEncoder LeftEncoder = MotorLeftM.getEncoder();


  public ChasisDriveSubsystem() {
    //Se resetea variables y encoders
    RightEncoder.setPosition(0);  RightEncoder.setPositionConversionFactor(6*Math.PI*(1/10.71));
    LeftEncoder.setPosition(0);   LeftEncoder.setPositionConversionFactor(6*Math.PI*(1/10.71));
    
    
    //Set motores en 0 y designamos motores esclavos e inversos
    MotorLeftS.follow(MotorLeftM);    MotorLeftM.set(0);
    MotorRightS.follow(MotorRightM);  MotorRightM.set(0); 

    //Configura todos los motores en Brake mode al recibir 0
    MotorLeftS.setIdleMode(CANSparkMax.IdleMode.kBrake);     MotorLeftM.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MotorRightS.setIdleMode(CANSparkMax.IdleMode.kBrake);    MotorRightM.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    //Reporte de valores chasis y joystick
    SmartDashboard.putNumber("Encoder Derecho", -RightEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Izquierdo", LeftEncoder.getPosition());
    SmartDashboard.putNumber("Velocidad Motor Derecho", MotorRightM.get());
    SmartDashboard.putNumber("Velocidad Motor Izquierdo", MotorLeftM.get());
  }

  public void SetMotors(double LeftSpeed, double RightSpeed){
    MotorLeftM.set(-LeftSpeed*.7); MotorRightM.set(-RightSpeed*.7);
  }
  
  public double getRightEncoder(){ return -RightEncoder.getPosition();}
  public double getLeftEncoder(){ return LeftEncoder.getPosition();}
    

  
  public void resetEncode(){
    RightEncoder.setPosition(0); LeftEncoder.setPosition(0);
  } 

  
}
