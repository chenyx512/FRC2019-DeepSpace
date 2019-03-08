package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Control;

public class Climber extends Subsystem {
    private DoubleSolenoid front, back;
    public WPI_TalonSRX masterTalon, slaveTalon;
    private Control control=Control.getInstance();
    private boolean isEndGame=false;

    @Override
    public void initDefaultCommand() {
    }

    public Climber(){
        front=new DoubleSolenoid(1,2,3);
        back=new DoubleSolenoid(1,0,1);
        masterTalon=new WPI_TalonSRX(9);//TODO look at ports
        slaveTalon=new WPI_TalonSRX(10);
        slaveTalon.follow(masterTalon);
        masterTalon.setInverted(true);
        slaveTalon.setInverted(false);
    }

    @Override
    public void periodic(){
        if(control.isAllDown()){
            isEndGame=true;
            front.set(Value.kForward);
            back.set(Value.kForward);
        }else{
            back.set(control.isBackUp()? Value.kReverse:Value.kOff);
            front.set(control.isFrontUp()? Value.kReverse:Value.kOff);
        }
        if(isEndGame)
            masterTalon.set(control.getForwardThrottle()*3);
    }

    public void configTalon(WPI_TalonSRX talon){
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
    }
}
