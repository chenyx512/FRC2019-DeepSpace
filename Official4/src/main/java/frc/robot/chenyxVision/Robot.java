/**
 * This is how we use the auto command,
 * COMMON SENSE: if you don't think auto if working, kill it
 * 
 * this has happened to us about only twice, BUT
 * the vision object may lose track of the target, if there is a big shake like going down the platform,
 * and then the auto command will start going based on the last known location of the target,
 * if you realize it and your robot is far from the target, restart the auto command (cancle and start),
 * otherwise, let go
 */


public class Robot extends TimedRobot {
    private static AutoRun autoRun = new AutoRun();

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
        else if(control.isStartAuto())
                autoRun.start();
        Scheduler.getInstance().run();
    }
}
