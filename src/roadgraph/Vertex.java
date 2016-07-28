package roadgraph;

import geography.GeographicPoint;
import java.lang.*;

/**
 * Created by nfrik on 7/22/16.
 */
public class Vertex{

    private GeographicPoint geographicPoint;

    private double distanceToStart  = Double.MAX_VALUE;

    private int id;

    public Vertex(GeographicPoint gp, double distanceToStart){
        setGeographicPoint(gp);
        setDistanceToStart(distanceToStart);
        id+=1;
    }

    public GeographicPoint getGeographicPoint() {
        return geographicPoint;
    }

    public void setGeographicPoint(GeographicPoint geographicPoint) {
        this.geographicPoint = geographicPoint;
    }

    public double getDistanceToStart() {
        return distanceToStart;
    }

    public void setDistanceToStart(double distanceToStart) {
        this.distanceToStart = distanceToStart;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    @Override
    public boolean equals(Object v){
        if(v instanceof Vertex){
            return this.geographicPoint.equals(((Vertex) v).geographicPoint);
        }else if (v instanceof GeographicPoint){
            return this.geographicPoint.equals((GeographicPoint) v);
        }

        return false;
    }


    @Override
    public int hashCode() {
        return geographicPoint.hashCode();
    }
}
