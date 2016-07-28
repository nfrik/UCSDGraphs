package roadgraph;

import geography.GeographicPoint;

/**
 * Created by nfrik on 7/14/16.
 */
public class Edge {

    private String roadType;

    private double streetLength;

    private GeographicPoint endVertex;


    public GeographicPoint getEndVertex() {
        return endVertex;
    }

    public void setEndVertex(GeographicPoint endVertex) {
        this.endVertex = endVertex;
    }

    public String getRoadType() {
        return roadType;
    }

    public void setRoadType(String streetName) {
        this.roadType = streetName;
    }

    public double getStreetLength() {
        return streetLength;
    }

    public void setStreetLength(double streetLength) {
        this.streetLength = streetLength;
    }




}
