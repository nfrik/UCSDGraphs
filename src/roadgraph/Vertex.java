package roadgraph;

import geography.GeographicPoint;

/**
 * Created by nfrik on 7/22/16.
 */
public class Vertex {

    private GeographicPoint geographicPoint;

    private double distanceToStart;

    public Vertex(GeographicPoint gp, double distanceToStart){
        setGeographicPoint(gp);
        setDistanceToStart(distanceToStart);
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



}
