package frc.robot.commands;
import frc.robot.subsystems.ChasisDriveSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveJoystickCmmd extends CommandBase {
  private final ChasisDriveSubsystem ChasisSub;
  private final Supplier<Double> speedFunction, turnFunction, reverseFunction;
  private final Supplier<Boolean> SpeedControlFunction;

  public DriveJoystickCmmd(ChasisDriveSubsystem ChasisSub, Supplier<Double> speedFunction, Supplier<Double> reverseFunction, Supplier<Double> turnFunction,Supplier<Boolean> SpeedControlFunction){
    addRequirements(ChasisSub);
    this.ChasisSub = ChasisSub;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    this.reverseFunction = reverseFunction;
    this.SpeedControlFunction = SpeedControlFunction;  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double MotorChasisSpeed = -speedFunction.get() - reverseFunction.get();
    double MotorChasisTurn = turnFunction.get();
    if(Math.abs(MotorChasisTurn)<=0.25){MotorChasisTurn = 0;}

    double SpeedR, SpeedL, MaxOutput;
    if(SpeedControlFunction.get()==true){MaxOutput = .4;}else{MaxOutput = 1;}

    if(Math.abs(MotorChasisSpeed) > 0.15){
      SpeedL = (MotorChasisSpeed-MotorChasisTurn)* MaxOutput; 
      SpeedR = (MotorChasisSpeed+MotorChasisTurn)*MaxOutput;
    }else if(Math.abs(MotorChasisTurn) > 0.25 && Math.abs(MotorChasisSpeed) <=0.15 ){
      SpeedL = -MotorChasisTurn* MaxOutput; 
      SpeedR = MotorChasisTurn*MaxOutput;
    }else{SpeedL = 0; SpeedR = 0;}

    ChasisSub.SetMotors(SpeedL, SpeedR);
    
  }

  @Override
  public void end(boolean interrupted) {
    ChasisSub.SetMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;}
}
