/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.lang.reflect.Array;
import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import gmapsfx.javascript.object.GoogleMap;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph{
	//TODO: Add your member variables here in WEEK 2


	private Map<GeographicPoint,ArrayList<Edge>> adjMapList;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjMapList = new HashMap<GeographicPoint,ArrayList<Edge>>();

	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return adjMapList.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> intersections = new HashSet<GeographicPoint>();

		for(GeographicPoint gp : adjMapList.keySet()){
			intersections.add(gp);
		}

		return intersections;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int ne=0;
		//TODO: Implement this method in WEEK 2
		for(GeographicPoint gp : adjMapList.keySet()){
			for(Edge gpe : adjMapList.get(gp)){
				ne++;
			}
		}
		return ne;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{

		if(!adjMapList.containsKey(location)){
			adjMapList.put(location,new ArrayList<Edge>());
			return true;
		}
		// TODO: Implement this method in WEEK 2
		return false;
	}

	public boolean updateVertex(GeographicPoint location){
		if(adjMapList.containsKey(location)){
			ArrayList<Edge> tempList = adjMapList.get(location);
			adjMapList.put(location,tempList);
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2

		if(length<0 || !adjMapList.containsKey(from) || !adjMapList.containsKey(to)) {
			throw new IllegalArgumentException();
		}else{
			ArrayList<Edge> tempList = adjMapList.get(from);
			Edge se = new Edge();

			se.setEndVertex(to);
			se.setRoadType(roadType);
			se.setStreetLength(length);

			tempList.add(se);

			adjMapList.put(from,tempList);
		}

	}

	public List<GeographicPoint> outNeighbors(GeographicPoint of){
		List<GeographicPoint> outNbs = new ArrayList<GeographicPoint>();
		if(!adjMapList.containsKey(of)){
			return null;
		}

		for(Edge se : adjMapList.get(of)){
			outNbs.add(se.getEndVertex());
		}
		return outNbs;
	}

	public List<GeographicPoint> inNeighbors(GeographicPoint of){
		List<GeographicPoint> inNbs = new ArrayList<GeographicPoint>();

		if(!adjMapList.containsKey(of)){
			return null;
		}

		for(GeographicPoint start : adjMapList.keySet()){
			for(Edge se : adjMapList.get(start)){
				if(se.getEndVertex().equals(of)) {
					inNbs.add(se.getEndVertex());
				}
			}
		}

		return inNbs;
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		List<GeographicPoint> gps = new ArrayList<>();
		gps.add(new GeographicPoint(12.3,34.5));
        Consumer<GeographicPoint> temp = (x) -> {new GeographicPoint(324,34);};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2

		Queue queue = new LinkedList<GeographicPoint>();
		Set visited = new HashSet<GeographicPoint>();
		GeographicPoint curr;

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();

		queue.add(start);
		visited.add(start);

		while(!queue.isEmpty()){
			curr = (GeographicPoint) queue.remove();

			nodeSearched.accept(curr);

			if(curr.equals(goal)){

				return getParentList(start, goal, parentMap);

			}

			for(GeographicPoint outNeighbor : outNeighbors(curr)){
				if(!visited.contains(outNeighbor)){
					visited.add(outNeighbor);
					queue.add(outNeighbor);

					parentMap.put(outNeighbor,curr);

				}
			}

		}


		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return null;
	}

	private List<GeographicPoint> getParentList(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> parentList = new ArrayList<GeographicPoint>();
		GeographicPoint currentNode = goal;
		while(currentNode!=start){
            parentList.add(currentNode);
            currentNode=parentMap.get(currentNode);
        }
		parentList.add(start);
		Collections.reverse(parentList);
		return parentList;
	}

	private List<GeographicPoint> getParentList(Vertex start, Vertex goal, Map<Vertex, Vertex> parentMap) {
		List<GeographicPoint> parentList = new ArrayList<GeographicPoint>();
		Vertex currentNode = goal;
		while(!currentNode.getGeographicPoint().equals(start.getGeographicPoint())){
			parentList.add(currentNode.getGeographicPoint());
			currentNode=parentMap.get(currentNode);
		}
		parentList.add(start.getGeographicPoint());
		Collections.reverse(parentList);
		return parentList;
	}

	private List<GeographicPoint> getParentList(GeographicPoint start, GeographicPoint goal) {
		List<GeographicPoint> parentList = new ArrayList<GeographicPoint>();
		GeographicPoint currentNode = goal;

		while(!currentNode.equals(start) && currentNode.getParent()!=null){
			parentList.add(currentNode);
			currentNode=currentNode.getParent();
		}
		parentList.add(start);
		Collections.reverse(parentList);
		return parentList;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<GeographicPoint> queue = new PriorityQueue<GeographicPoint>(new Comparator<GeographicPoint>() {
			@Override
			public int compare(GeographicPoint V1, GeographicPoint V2) {
				return (V1.getDistanceToStart()==V2.getDistanceToStart() ? 0 : (V1.getDistanceToStart()<V2.getDistanceToStart()) ? -1 : 1);
			}
		});

		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();

		GeographicPoint currVtx; // We use vertex in order to keep two values in one object (GeographicPoint and Priority)

//		Vertex tempNeighbor;

		queue.add(new GeographicPoint(start,0));

		while (!queue.isEmpty()){
			currVtx = queue.remove();

			nodeSearched.accept(currVtx);

			if(!visited.contains(currVtx)){
				visited.add(currVtx);



				if(currVtx.equals(goal)){
					return getParentList(start,currVtx);
				}

				for(GeographicPoint neighbor : outNeighbors(currVtx)){

					//if neighbor is in visited list, check if distance to start is shorter
					//else create one with infinite distance to start

					double dist = currVtx.getDistanceToStart()+currVtx.distance(neighbor);

					if(dist < neighbor.getDistanceToStart()) {

						neighbor.setDistanceToStart(currVtx.getDistanceToStart() + dist);

						neighbor.setParent(currVtx);

//						updateVertex(neighbor);
						//new Vertex(neighbor,currVtx.getGeographicPoint().distance(neighbor.getGeographicPoint()));

//						parentList.add(neighbor);

//						parentMap.put(neighbor, currVtx);

						queue.add(neighbor);
					}

				}

			}
		}



		// Hook for visualization.  See writeup.
		// nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<GeographicPoint> queue = new PriorityQueue<GeographicPoint>(new Comparator<GeographicPoint>() {
			@Override
			public int compare(GeographicPoint V1, GeographicPoint V2) {
				return (V1.getDistanceToStart()==V2.getDistanceToStart() ? 0 : (V1.getDistanceToStart()<V2.getDistanceToStart()) ? -1 : 1);
			}
		});

		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();

		GeographicPoint currVtx; // We use vertex in order to keep two values in one object (GeographicPoint and Priority)

//		Vertex tempNeighbor;

		queue.add(new GeographicPoint(start,0));

		while (!queue.isEmpty()){
			currVtx = queue.remove();

			nodeSearched.accept(currVtx);

			if(!visited.contains(currVtx)){
				visited.add(currVtx);



				if(currVtx.equals(goal)){
					return getParentList(start,currVtx);
				}

				for(GeographicPoint neighbor : outNeighbors(currVtx)){

					//if neighbor is in visited list, check if distance to start is shorter
					//else create one with infinite distance to start

					double dist = currVtx.getDistanceToStart()+currVtx.distance(neighbor);

					if(dist < neighbor.getDistanceToStart()) {

						neighbor.setDistanceToStart(currVtx.getDistanceToStart() + dist + neighbor.distance(goal));

						neighbor.setParent(currVtx);

//						updateVertex(neighbor);
						//new Vertex(neighbor,currVtx.getGeographicPoint().distance(neighbor.getGeographicPoint()));

//						parentList.add(neighbor);

//						parentMap.put(neighbor, currVtx);

						queue.add(neighbor);
					}

				}

			}
		}



		// Hook for visualization.  See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	/**
	 * Generate string representation of adjacency list
	 * @return the String
	 */
	public String adjacencyString() {
		String s = "Adjacency list";
		s += " (size " + getNumVertices() + "+" + getNumEdges() + " integers):";

		for (GeographicPoint v : adjMapList.keySet()) {
			s += "\n\t"+v+": ";
			for (Edge w : adjMapList.get(v)) {
				s += w.getEndVertex()+", ";
			}
		}
		return s;
	}

	
	
	public static void main(String[] args)
	{

		System.out.println();

		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);

		Set<GeographicPoint> vertSet = firstMap.getVertices();
		List<GeographicPoint> vertList = new ArrayList<GeographicPoint>(vertSet);

//		GeographicPoint start = vertList.get(0);
//
//		GeographicPoint goal = vertList.get(vertList.size()-1);

		GeographicPoint start = new GeographicPoint(7,3);

		GeographicPoint goal = new GeographicPoint(4,-1);

		List<GeographicPoint> parentList = firstMap.dijkstra(start,goal);

		System.out.println(parentList);

//		System.out.println(firstMap.inNeighbors(start));
//
//		System.out.println(firstMap.outNeighbors(start));



//		System.out.print(firstMap.adjacencyString());
//		System.out.println();
//		System.out.print(firstMap.bfs(new GeographicPoint(1.0,1.0),new GeographicPoint(4.0,2.0)));
		System.out.println("DONE.");

		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		GraphLoader.loadRoadMap("data/testdata/map1.txt", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(7.0, 3.0);
		GeographicPoint testEnd = new GeographicPoint(4.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
