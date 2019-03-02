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
    private Joystick operate=new Joystick(1);

    public boolean isEstop() {
        if(operate!=null)
            return operate.getRawButton(2) || drive.getRawButton(1);
        return drive.getRawButton(1);
    }
    
    //drivetrain starts
    public boolean isQuickTurn(){return drive.getRawButton(2);}
    public double getForwardThrottle() {return drive.getRawAxis(1) * -1;}
    public double getRotationThrottle() {return drive.getRawAxis(0);}
    public double getSpeed() {
        return 0.6;
        // return (1 - drive.getRawAxis(3)) / 2;
    }

    //HatchArm Starts
    public boolean isChangeHatchGrabber() {return drive.getRawButtonPressed(6);}
    public boolean isChangeDrawSlide() {return drive.getRawButtonPressed(4);}
    public double getOverrideArm(){
        if(operate!=null){
            switch(operate.getPOV()){
                case 0: return 1;
                case 180: return -1;
            }
        }
        switch(drive.getPOV()){
            case 0: return 1;
            case 180: return -1;
            default: return 0;
        }
    }
    /** @return -1 nothing, 0-2 as the level */
    public int getArmPosition(){
        if(drive.getRawButton(11))
            return 0;
        if(drive.getRawButton(9))
            return 1;
        if(drive.getRawButton(7))
            return 2;
        return -1;
    }

    //climber starts
    public boolean isFrontUp(){
        if(operate==null)
            return false;
        else
            return operate.getRawButtonPressed(5);
    }
    public boolean isBackUp(){
        if(operate==null)
            return false;
        else
            return operate.getRawButtonPressed(6);
    }
    public boolean isAllDown(){
        if(operate==null)
            return false;
        else
            return operate.getRawButtonPressed(4);
    }
    public boolean isEndGame(){
        if(operate==null)
            return false;
        else 
            return operate.getRawButton(7);
    }

    //auto starts
    public double getAutoAngleOverride(){
        if(operate==null)
            return 0;
        double l=operate.getRawAxis(2), r=operate.getRawAxis(3);
        return l<0.05? r:-l;
    }
    public double getAutoSpeedCo(){
        if(operate==null)
            return 1;
        else
            return (2-operate.getRawAxis(1))/2;
    }
    public boolean getAutoShoot(){
        return operate==null? false:operate.getRawButton(6);
    }
    public boolean isAutoClose(){
        return operate==null? false:operate.getRawButton(5);
    }
    public boolean isStartAuto(){
        if(operate!=null)
            return operate.getRawButton(1)||drive.getRawButton(3);
        return drive.getRawButton(3);
    }
    public boolean isLockTarget(){
        if(operate!=null)
            return operate.getRawButton(3)||drive.getRawButton(5);
        return drive.getRawButton(5);
    }
}