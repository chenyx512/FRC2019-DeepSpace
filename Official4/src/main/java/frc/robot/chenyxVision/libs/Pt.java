package frc.robot.chenyxVision.libs;

/**
 * the same as Vector, because java has no typedef....
 */
public class Pt extends Vector{
    public Pt(double _x, double _y){
        super(_x, _y);
    }
    public Pt(Vector other){
        super(other);
    }
}