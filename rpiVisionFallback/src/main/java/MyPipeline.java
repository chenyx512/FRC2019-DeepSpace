// import edu.wpi.cscore.VideoSource;
// import edu.wpi.first.vision.VisionPipeline;

// import java.util.*;
// import org.opencv.core.*;
// import org.opencv.imgproc.*;

// //always remember to release memory, and make sure the thing you call release isn't NULL
// public class MyPipeline implements VisionPipeline {
//     private VideoSource camera;
//     private Mat hsv=new Mat(), hsvOutput=new Mat();
//     private int rectCnt, smallIgnoreCnt, angleIgnoreCnt;
//     private ArrayList<RotatedRect> leftRects = new ArrayList<RotatedRect>(), rightRects = new ArrayList<RotatedRect>();
//     private ArrayList<MatOfPoint> contours=new ArrayList<MatOfPoint>();
//     private boolean isInit=false;

//     public double lowH = Constants.LOW_H, highH = Constants.HIGH_H, lowS = Constants.LOW_S;
//     public double lowV = Constants.LOW_V, highV = Constants.HIGH_V, highS = Constants.HIGH_S;
//     public Output output=new Output();
//     public double width, height;
//     public Mat maskOutput;
//     public int outputCnt = 0;
    
//     public MyPipeline(VideoSource _camera){
//         camera=_camera;
//     }

//     @Override
//     public void process(Mat source) {
//         if(!isInit){
//             width=source.width();
//             height=source.height();
//             Constants.FOCAL_LENGTH_HORIZONTAL=width/2/(Math.tan(Math.toRadians(Constants.HORIZONTAL_FOV_DEG/2)));
//             Constants.FOCAL_LENGTH_VERTICAL=height/2/(Math.tan(Math.toRadians(Constants.VERTICAL_FOV_DEG/2)));
//             isInit=true;
//         }

//         output=new Output();
//         output.timestamp = camera.getLastFrameTime()/1000 - Main.startTime;
//         if(maskOutput!=null)maskOutput.release();
//         maskOutput = source.clone();
//         leftRects = new ArrayList<RotatedRect>();
//         rightRects = new ArrayList<RotatedRect>();
//         contours = new ArrayList<MatOfPoint>(); //TODO clean this
//         rectCnt = smallIgnoreCnt = angleIgnoreCnt = 0;

//         // hsv threshhold to U8C1
//         Imgproc.cvtColor(source, hsv, Imgproc.COLOR_BGR2HSV);
//         Core.inRange(hsv, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), hsvOutput);
//         // find contour
//         Imgproc.findContours(hsvOutput, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//         // find min-area bonding rect
//         for (int i = 0; i < contours.size(); i++) {
//             if (Imgproc.contourArea(contours.get(i)) < Constants.MIN_DETECT_AREA) {
//                 smallIgnoreCnt++;
//                 continue;
//             }
//             RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
//             if(Math.max(rect.size.width, rect.size.height)/Math.min(rect.size.width, rect.size.height)>Constants.MAX_W2H_RATIO){
//                 continue;
//             }
//             double rectAngle = rect.size.width > rect.size.height ? rect.angle + 90 : rect.angle;
//             if (Math.abs(rectAngle) < Constants.MIN_DETECT_ANGLE || Math.abs(rectAngle) > Constants.MAX_DETECT_ANGLE) {
//                 angleIgnoreCnt++;
//                 continue;
//             }
//             if (rectAngle < 0)
//                 rightRects.add(rect);
//             else
//                 leftRects.add(rect);
//             rectCnt++;
//             drawRotatedRect(maskOutput, rect, rectAngle < 0 ? new Scalar(255, 0, 0) : new Scalar(0, 0, 255), 2);
//         }

//         if(leftRects.size()>5 || rightRects.size()>5)
//             return;
//         for (RotatedRect lRect : leftRects)
//             for (RotatedRect rRect : rightRects) {
//                 if (lRect.center.x > rRect.center.x)
//                     continue;
//                 double ratioDx2Length = (rRect.center.x - lRect.center.x) / (Math.max(lRect.size.height, lRect.size.width));
//                 double ratioDy2Length = Math.abs((rRect.center.y - lRect.center.y) / Math.max(lRect.size.height, lRect.size.width));
//                 if (ratioDx2Length < Constants.MAX_DX2LENGTH && ratioDy2Length < Constants.MAX_DY2LENGTH) {
//                     Point leftPt=getLeftOrRightPoint(lRect, false), rightPt=getLeftOrRightPoint(rRect, true);
//                     double x=(leftPt.x+rightPt.x)/2, y=(leftPt.y+rightPt.y)/2;
//                     Point angleL = calculateAngle(getLeftOrRightPoint(lRect, false));
//                     output.leftX.add(angleL.x);
//                     Point angleR = calculateAngle(getLeftOrRightPoint(rRect, true));
//                     output.rightX.add(angleR.x);
//                     output.leftHeight.add(getHeight(lRect)/height);
//                     output.rightHeight.add(getHeight(rRect)/height);
//                     Imgproc.circle(maskOutput, new Point(x, y), 3, new Scalar(255, 0, 0), 3);
//                     break;
//                 }
//             }
//         //debugDraw();
//         outputCnt++;
//     }

//     private Point getLeftOrRightPoint(RotatedRect rect, boolean isGetLeft){
//         Point[] vertices = new Point[4];
//         rect.points(vertices);
//         Point RV=vertices[0];
//         for(int i=1;i<=3;i++)
//             if(isGetLeft? vertices[i].x<RV.x : vertices[i].x > RV.x)
//                 RV=vertices[i];
//         return RV;
//     }
//     private double getHeight(RotatedRect rect){
//         Point[] vertices = new Point[4];
//         rect.points(vertices);
//         double max=vertices[0].y, min=vertices[0].y;
//         for(int i=0;i<=3;i++){
//             max = Math.max(max, vertices[i].y);
//             min = Math.min(min, vertices[i].y);
//         }
//         return max-min;
//     }

//     //based on pin hole model, RV is angle in degree
//     private Point calculateAngle (Point pt){
//         double x=pt.x, y=pt.y;
//         return new Point(Math.toDegrees(Math.atan((x-width/2)/Constants.FOCAL_LENGTH_HORIZONTAL)),
//             Math.toDegrees(Math.atan((y-height/2)/Constants.FOCAL_LENGTH_VERTICAL)) );
//     }

//     private void debugDraw() {
//         Imgproc.putText(maskOutput, String.format("%d %d %d", rectCnt, angleIgnoreCnt, smallIgnoreCnt),
//                 new Point(10, 50), // point
//                 Core.FONT_HERSHEY_SIMPLEX, // front face
//                 1, // front scale
//                 new Scalar(255, 0, 0), // Scalar object for color
//                 4 // Thickness
//         );
//     }

//     private static void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color, int thickness) {
//         Point[] vertices = new Point[4];
//         rotatedRect.points(vertices);
//         MatOfPoint points = new MatOfPoint(vertices);
//         Imgproc.drawContours(image, Arrays.asList(points), -1, color, thickness);
//     }
// }