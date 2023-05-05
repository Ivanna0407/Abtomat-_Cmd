package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChasisDriveSubsystem;

public class FowardChasisPidCmmd extends CommandBase {
  private final ChasisDriveSubsystem ChasisSub;
  private final double Setpoint;
  private double IntegralZone;

  //Kp = número proporcional 
  private final double Kp = 0.025; private double P_ErrorR = 0; 
  private double P_ErrorL = 0; private double PosicionR = 0; private double PosicionL = 0;
  //Ki = número integral
  private final double Ki = 0.025; private double I_ErrorR = 0; private double I_ErrorL = 0; private double LastTime = 0;
  //kD = número derivativo
  private final double Kd = 0.002; private double D_ErrorR = 0; private double D_ErrorL = 0; private double LastErrorR = 0;  private double LastErrorL = 0;
  
  


  public FowardChasisPidCmmd(ChasisDriveSubsystem ChasisSub, double Setpoint) {
    this.ChasisSub = ChasisSub;
    this.Setpoint = Setpoint;
    addRequirements(ChasisSub);
  }

  
  @Override
  public void initialize() {
    P_ErrorR = 0; I_ErrorR = 0; D_ErrorR = 0; 
    P_ErrorL = 0; I_ErrorL = 0; D_ErrorL = 0;
    LastTime = 0; LastErrorR = 0; LastErrorL = 0;
    ChasisSub.resetEncode();
    IntegralZone = Math.abs((Setpoint - ChasisSub.getLeftEncoder() ) *.1);
  }

  
  @Override
  public void execute() {
    PosicionR = ChasisSub.getRightEncoder();
    PosicionL = ChasisSub.getLeftEncoder();

    //Proportional error
    P_ErrorR = Setpoint - PosicionR;
    P_ErrorL = Setpoint - PosicionL;

    //Integral error
    double dt = Timer.getFPGATimestamp() - LastTime;
    if(Math.abs(P_ErrorR) < IntegralZone){I_ErrorR += P_ErrorR*dt;}
    if(Math.abs(P_ErrorL) < IntegralZone){I_ErrorL += P_ErrorL*dt;}

    //Derivative error
    if(Math.abs(P_ErrorR) < IntegralZone){D_ErrorR = (P_ErrorR - LastErrorR) / dt;}
    if(Math.abs(P_ErrorL) < IntegralZone){D_ErrorL = (P_ErrorL - LastErrorL) / dt;}
     
    Double SpeedR = Kp*P_ErrorR + Ki*I_ErrorR + Kd*D_ErrorR;
    Double SpeedL = Kp*P_ErrorL + Ki*I_ErrorL + Kd*D_ErrorL;

    ChasisSub.SetMotors(-SpeedL, -SpeedR); 
    LastTime = Timer.getFPGATimestamp();
    LastErrorR = P_ErrorR; LastErrorL = P_ErrorL;
  }

 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    if(Math.abs(Setpoint-PosicionR) <= Setpoint*.01 && Math.abs(Setpoint-PosicionL) <= Setpoint*.01){
      ChasisSub.SetMotors(0, 0);
      return true;}
    else{return false;}
  }
}
