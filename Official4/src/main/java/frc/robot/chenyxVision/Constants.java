package frc.robot.chenyxVision;
import frc.robot.chenyxVision.libs.Pt;

public class Constants {
    // localization and vision constants
    public static final Pt cameraPt = new Pt(100, 0);

    public static final double VISION_DISTANCE_CONSTANT = 211; //actualDis*imageHeight (mm*px)
    public static final int MAX_HISTORY_SIZE = 150;
    public static final int HUD_WIDTH=320;
    public static final int HUD_HEIGHT=240;
    public static final double HUD_HORIZONTAL_FOV=63;
    public static final double ROBOT_STATE_PERIOD=0.01;
    public static final double LOCK_ERROR=170; //mm
    public static final double HUD_MAX_SHOWN_DISTANCE=5000; //mm
    public static final double VISION_DISCONNECT_TIME=2;

    //auto constants
    public static final double MIN_TURN_SPEED=0.12; //rotational pwr, TODO tune this
    public static final double MAX_ALLOWED_ANGLE_ERROR = 2.5; //deg

    public static final double SHOOT_DIS=180; //mm, TODO tune this
    public static final double CLOSE_DIS=SHOOT_DIS + 700; //mm, TODO tune this according to above
    public static final double START_ADDITION_SPEED=0.25; //linear pwr
    public static final double CLOSE_SPEED=0.26; //linear pwr
    public static final double CLOSE_ANGULAR_ERROR_PENALTY=0.0135; // deg/linear_pwr

    public static final double PHYSICAL_OFFSET=0; // adjust it if camera is not centered
    public static final double OFFSET_THRESH=15; 
    public static final double OFFSET_COMP=0; // use only if you delivery system requires
}