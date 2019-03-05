package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Control{
    private static Control instance;
    public static Control getInstance() {
        if(instance==null)
            synchronized(Control.class){
                if(instance==null)
                    instance=new Control();
            }
        return instance;
    }
    
    private Joystick drive=new Joystick(0);
    private Joystick operate=null;

    public boolean isEstop() {
        return drive.getRawButton(3);
    }
    
    //drivetrain starts
    public boolean isQuickTurn(){return drive.getRawButton(2);}
    public double getForwardThrottle() {return drive.getRawAxis(1) * -1;}
    public double getRotationThrottle() {return drive.getRawAxis(0);}

    //HatchArm Starts
    public boolean isChangeHatchGrabber() {return drive.getRawButtonPressed(6);}
    public boolean isChangeDrawSlide() {return drive.getRawButtonPressed(4);}

    //climber starts
    public boolean isFrontUp(){
        return drive.getRawButton(9);
    }
    public boolean isBackUp(){
        return drive.getRawButton(11);
    }
    public boolean isAllDown(){
        return drive.getRawButton(7);
    }

    //auto starts
    public double getAutoAngleOverride(){
        return 0;
        // if(operate==null)
        //     return getRotationThrottle();
        // double l=operate.getRawAxis(2), r=operate.getRawAxis(3);
        // return l<0.05? r:-l;
    }
    public double getAutoSpeedCo(){
        // if(operate==null)
        //     return 1+getForwardThrottle();
        return 1;
    }
    public boolean getAutoShoot(){
        return drive.getRawButton(8);
        // return operate==null? false:operate.getRawButton(6);
    }
    public boolean isStartAutoGet(){
        if(operate!=null)
            return operate.getRawButton(1)||drive.getRawButton(3);
        return drive.getRawButton(3);
    }
    public boolean isStartAutoPost(){
        if(operate!=null)
            return operate.getRawButton(1)||drive.getRawButton(5);
        return drive.getRawButton(5);
    }
    public boolean isLockTarget(){
        if(operate!=null)
            return operate.getRawButton(3)||drive.getRawButton(10);
        return drive.getRawButton(10);
    }
}