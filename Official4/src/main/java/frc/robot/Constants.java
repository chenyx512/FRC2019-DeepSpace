package frc.robot;

/*
drive 1,2,3,4
lower 5,6
arm 7,8
*/

public class Constants{
    // localization and vision constants
    public static final double VISION_DISTANCE_CONSTANT = 211; //actualDis*imageHeight (mm*px)
    public static final int MAX_HISTORY_SIZE = 150;
    public static final int HUD_WIDTH=320;
    public static final int HUD_HEIGHT=240;
    public static final double HUD_HORIZONTAL_FOV=125;
    public static final double ROBOT_STATE_PERIOD=0.01;
    public static final double LOCK_ERROR=170; //mm
    public static final double HUD_MAX_SHOWN_DISTANCE=5000; //mm
    public static final double VISION_DISCONNECT_TIME=4;
    public static final double LOCK_MAX_UPDATE_DURATION=5; //sec

    //auto constants
    public static final double SHOOT_DIS=180; //mm
    public static final double CLOSE_DIS=800; //mm
    public static final double CLOSE_SPEED=0.26; //linear pwr
    public static final double OFFSET_THRESH=15;
    public static final double OFFSET_COMP=2;
    public static final double START_ADDITION_SPEED=0.25; //linear pwr
    public static final double CLOSE_ANGULAR_ERROR_PENALTY=0.0135; // deg/linear_pwr
    public static final double PHYSICAL_OFFSET=1;

    //Talon DRIVETRAIN
    public static final double DRIVETRAIN_ENCODER_K=0.3015; // actual:read, 0.4854
    public static final double DRIVETRAIN_DEFAULT_OPEN_RAMP=0.55;
    public static final int DRIVE_PEAK_CURRENT_DURANTION = 100; //ms
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 50; //amps
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 20; //amps
    public static final double TALON_MAX_VOLTAGE = 12.5;
    public static final Gains DRIVETRAIN_VELOCITY_GAINS=new Gains(3.5,0.01,40,1023/340.0,20,1);
    public static final int DRIVETRAIN_VELOCITY_SLOT=0;
    // public static final double Aff = 1023/4000;
    //ARM
    public static final Gains ARM_POSITION_GAINS=new Gains(80,0.06,1000,0,3,1);
    public static final int ARM_POSITION_SLOT=0;
    public static final double DEFAULT_CLOSED_RAMP=0.75;
    public static final double[] ARM_POSITION = {889,938,961};

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;
        
        public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = _kIzone;
            kPeakOutput = _kPeakOutput;
        }
    }
}