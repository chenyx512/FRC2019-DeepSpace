package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoRun;
import frc.robot.libs.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
    public static Climber climber = new Climber();
    public static DriveTrain driveTrain = new DriveTrain();
    public static HatchArm hatchArm = new HatchArm();
    public static int loopCnt=0;

    public static Compressor compressor = new Compressor(1);
    private static RobotState robotState=RobotState.getInstance();
    private static Control control = Control.getInstance();
    private static HUD hud= HUD.getInstance();
    private static Vision vision=Vision.getInstance();
    private static AutoRun autoRun = new AutoRun();

    @Override
    public void robotInit() {
        compressor.setClosedLoopControl(true);
    }

    @Override
    public void robotPeriodic() {
        loopCnt++;
        if((loopCnt&7)==0)
            dashboardDisplay();
        if(!vision.isConnected)
            hud.messages[0]="NP";
        else if(!autoRun.isRunning())
            hud.messages[0]="M";
    }

    @Override
    public void autonomousPeriodic() {
        commonPeriodic();
    }
    @Override
    public void teleopPeriodic() {
        commonPeriodic();
    }

    private void commonPeriodic(){
        if(autoRun.isRunning()){
            if(control.isEstop())
                autoRun.cancel(); 
        }
        else{
            if(control.isStartAutoGet()){
                autoRun.isGet=true;
                autoRun.start();
            } else if(control.isStartAutoPost()){
                autoRun.isGet=false;
                autoRun.start();
            }
        }
        Scheduler.getInstance().run();
    }

    private void dashboardDisplay(){
        Pose pose=robotState.getPose();
        SmartDashboard.putNumber("x",pose.x);
        SmartDashboard.putNumber("y",pose.y);
        SmartDashboard.putNumber("theta",pose.theta);
    }
}
