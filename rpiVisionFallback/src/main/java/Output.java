import java.util.ArrayList;

public class Output {
    ArrayList<Double> leftX, rightX, leftHeight, rightHeight;
    long timestamp;

    public Output() {
        leftX = new ArrayList<>();
        rightX = new ArrayList<>();
        leftHeight = new ArrayList<>();
        rightHeight =  new ArrayList<>();
        timestamp = 0;
    }
}