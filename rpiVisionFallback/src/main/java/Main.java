import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;

// moves to the pipeline class
// import org.opencv.core.Mat;
// import edu.wpi.first.vision.VisionPipeline;

public final class Main {
    private static String configFile = "/boot/frc.json";

    @SuppressWarnings("MemberName")
    public static class CameraConfig {
        public String name;
        public String path;
        public JsonObject config;
    }

    public static int team;
    public static boolean server;
    public static List<CameraConfig> cameraConfigs = new ArrayList<>();
    private static final Object imgLock = new Object();// this is used to synchronize the getting and sending of data in
                                                       // different threads
    public static long startTime;

    public static void main(String... args){
        if (args.length > 0) {
            configFile = args[0];
        }
        if (!readConfig()) {
            return;
        }
        if(cameraConfigs.size()!=1){
           // throw new ChenyxCameraNumberNotOneException();
           System.out.printf("error");
        }

        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        if (server) {
            System.out.println("Setting up NetworkTables server");
            ntinst.startServer();
        } else {
            System.out.println("Setting up NetworkTables client for team" + team);
            ntinst.startClientTeam(team);
        }

        VideoSource camera=startCamera(cameraConfigs.get(0));
        CvSink sink=CameraServer.getInstance().getVideo(camera);
        NetworkTable outputTable=ntinst.getTable("piOutput");
        NetworkTableEntry outputEntry=outputTable.getEntry("output");
        ntinst.flush();
        ProcessThread.outputEntry=outputEntry;
        ProcessThread.ntinst=ntinst;
        ProcessThread[] processThreads=new ProcessThread[Constants.PROCESS_THREAD_NUMBER];
        Constants.startTime=System.currentTimeMillis()/1000.0;
        for(int cnt=1;;cnt++){
            Mat source=new Mat();
            double timestamp=sink.grabFrameNoTimeout(source)/1000000.0;
            if(cnt==1){
                Constants.FOCAL_LENGTH_HORIZONTAL=source.width()/2.0/(Math.tan(Math.toRadians(Constants.HORIZONTAL_FOV_DEG/2)));
                Constants.FOCAL_LENGTH_VERTICAL=source.height()/2.0/(Math.tan(Math.toRadians(Constants.VERTICAL_FOV_DEG/2)));
                Constants.HEIGHT=source.height();
                Constants.WIDTH=source.width();
                ProcessThread.outputStream=CameraServer.getInstance().putVideo("output", source.width(), source.height());
            }
            boolean isRun=false;
            for(int i=0;i<Constants.PROCESS_THREAD_NUMBER;i++)
                if(processThreads[i]==null || !processThreads[i].isAlive()){
                    processThreads[i] = new ProcessThread(i, source, timestamp);
                    processThreads[i].start();
                    isRun=true;
                    break;
                }
            if(!isRun){
                System.out.printf("all threads busy\n");
            }
        }
    }

    public static VideoSource startCamera(CameraConfig config) {
        System.out.println("Starting camera '" + config.name + "' on " + config.path);
        VideoSource camera = CameraServer.getInstance().startAutomaticCapture(config.name, config.path);
        Gson gson = new GsonBuilder().create();
        camera.setConfigJson(gson.toJson(config.config));
        return camera;
    }
    private Main() {
    }

    /**
     * Report parse error.
     */
    public static void parseError(String str) {
        System.err.println("config error in '" + configFile + "': " + str);
    }

    /**
     * Read single camera configuration.
     */
    public static boolean readCameraConfig(JsonObject config) {
        CameraConfig cam = new CameraConfig();

        // name
        JsonElement nameElement = config.get("name");
        if (nameElement == null) {
            parseError("could not read camera name");
            return false;
        }
        cam.name = nameElement.getAsString();

        // path
        JsonElement pathElement = config.get("path");
        if (pathElement == null) {
            parseError("camera '" + cam.name + "': could not read path");
            return false;
        }
        cam.path = pathElement.getAsString();

        cam.config = config;

        cameraConfigs.add(cam);
        return true;
    }

    /**
     * Read configuration file.
     */
    @SuppressWarnings("PMD.CyclomaticComplexity")
    public static boolean readConfig() {
        // parse file
        JsonElement top;
        try {
            top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
        } catch (IOException ex) {
            System.err.println("could not open '" + configFile + "': " + ex);
            return false;
        }

        // top level must be an object
        if (!top.isJsonObject()) {
            parseError("must be JSON object");
            return false;
        }
        JsonObject obj = top.getAsJsonObject();

        // team number
        JsonElement teamElement = obj.get("team");
        if (teamElement == null) {
            parseError("could not read team number");
            return false;
        }
        team = teamElement.getAsInt();

        // ntmode (optional)
        if (obj.has("ntmode")) {
            String str = obj.get("ntmode").getAsString();
            if ("client".equalsIgnoreCase(str)) {
                server = false;
            } else if ("server".equalsIgnoreCase(str)) {
                server = true;
            } else {
                parseError("could not understand ntmode value '" + str + "'");
            }
        }

        // cameras
        JsonElement camerasElement = obj.get("cameras");
        if (camerasElement == null) {
            parseError("could not read cameras");
            return false;
        }
        JsonArray cameras = camerasElement.getAsJsonArray();
        for (JsonElement camera : cameras) {
            if (!readCameraConfig(camera.getAsJsonObject())) {
                return false;
            }
        }

        return true;
    }
}
