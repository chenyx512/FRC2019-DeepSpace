package frc.robot.libs;

public class Vector {
    public double x,y;
    public Vector(double _x, double _y){
        x=_x;
        y=_y;
    }
    public Vector(Vector other){
        this(other.x, other.y);
    }

    public double distance(Vector other){
        return Math.sqrt((x-other.x)*(x-other.x)+(y-other.y)*(y-other.y));
    }

    public Vector plus(Vector other){
        return new Vector(x+other.x, y+other.y);
    }

    public Vector scale(double scale){
        return new Vector(x*scale, y*scale);
    }
}
