package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Control;
import frc.robot.HUD;

public class HatchArm extends Subsystem {
    private Solenoid hatchGrabber, slide;
    private Control control=Control.getInstance();
    private HUD hud=HUD.getInstance();
    public boolean isHatchGrabberOpen=true, isSlideForward=false;

    @Override
    public void initDefaultCommand() {
    }
    public HatchArm(){
        hatchGrabber=new Solenoid(0,1);
        slide=new Solenoid(0,2);
    }
    
    @Override
    public void periodic(){
        if(control.isChangeHatchGrabber())
            isHatchGrabberOpen^=true;
        if(control.isChangeDrawSlide())
            isSlideForward^=true;
        hatchGrabber.set(!isHatchGrabberOpen);
        slide.set(isSlideForward);
        hud.messages[2]=(isSlideForward? "--":"")+(isHatchGrabberOpen? "<":">");
    }
}
