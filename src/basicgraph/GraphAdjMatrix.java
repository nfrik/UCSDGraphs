package basicgraph;

import util.GraphLoader;

import java.util.*;

/** A class that implements a directed graph. 
 * The graph may have self-loops, parallel edges. 
 * Vertices are labeled by integers 0 .. n-1
 * and may also have String labels.
 * The edges of the graph are not labeled.
 * Representation of edges via an adjacency matrix.
 * 
 * @author UCSD MOOC development team and YOU
 *
 */
public class GraphAdjMatrix extends Graph {

	private final int defaultNumVertices = 5;
	private int[][] adjMatrix;
	
	/** Create a new empty Graph */
	public GraphAdjMatrix () {
		adjMatrix = new int[defaultNumVertices][defaultNumVertices];
	}
	
	/** 
	 * Implement the abstract method for adding a vertex.
	 * If need to increase dimensions of matrix, double them
	 * to amortize cost. 
	 */
	public void implementAddVertex() {
		int v = getNumVertices();
		if (v >= adjMatrix.length) {
			int[][] newAdjMatrix = new int[v*2][v*2];
			for (int i = 0; i < adjMatrix.length; i ++) {
				for (int j = 0; j < adjMatrix.length; j ++) {
					newAdjMatrix[i][j] = adjMatrix[i][j];
				}
			}
			adjMatrix = newAdjMatrix;
		}
	}
	
	/** 
	 * Implement the abstract method for adding an edge.
	 * Allows for multiple edges between two points:
	 * the entry at row v, column w stores the number of such edges.
	 * @param v the index of the start point for the edge.
	 * @param w the index of the end point for the edge.  
	 */	
	public void implementAddEdge(int v, int w) {
		adjMatrix[v][w] += 1;
	}
	
	/** 
	 * Implement the abstract method for finding all 
	 * out-neighbors of a vertex.
	 * If there are multiple edges between the vertex
	 * and one of its out-neighbors, this neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getNeighbors(int v) {
		List<Integer> neighbors = new ArrayList<Integer>();
		for (int i = 0; i < getNumVertices(); i ++) {
			for (int j=0; j< adjMatrix[v][i]; j ++) {
				neighbors.add(i);
			}
		}
		return neighbors;
	}
	
	/** 
	 * Implement the abstract method for finding all 
	 * in-neighbors of a vertex.
	 * If there are multiple edges from another vertex
	 * to this one, the neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */
	public List<Integer> getInNeighbors(int v) {
		List<Integer> inNeighbors = new ArrayList<Integer>();
		for (int i = 0; i < getNumVertices(); i ++) {
			for (int j=0; j< adjMatrix[i][v]; j++) {
				inNeighbors.add(i);
			}
		}
		return inNeighbors;
	}
	
	/** 
	 * Implement the abstract method for finding all 
	 * vertices reachable by two hops from v.
	 * Use matrix multiplication to record length 2 paths.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getDistance2(int v) {
		// XXX Implement this method in week 1
		int[][] newAdjMatrix = matrixMul(adjMatrix,adjMatrix);
		List<Integer> dist2vtxs=new ArrayList<Integer>();

//		for(int i=0;i<adjMatrix.length;i++){
//		 if(newAdjMatrix[i][v]>0){
//			 dist2vtxs.add(i);
//		 }
//		}

		for(int i=0;i<newAdjMatrix.length;i++){
			for(int m=0;m<newAdjMatrix[v][i];m++) {
				dist2vtxs.add(i);
			}
		}

		Collections.sort(dist2vtxs);

		return dist2vtxs;
	}

	public int[][] matrixMul(int[][] a, int[][] b){
		int[][] newAdjMatrix = new int[a.length][a.length];
		int temp;
		for(int i=0;i<a.length;i++){
			for(int j=0;j<a.length;j++){

				temp=0;
//				for(int k=i;k<adjMatrix.length;k++){
					for(int m=0;m<a.length;m++){
						temp+=a[i][m]*b[m][j];
					}
//				}
				newAdjMatrix[i][j]=temp;

			}
		}

		return newAdjMatrix;
	}
	
	/**
	 * Generate string representation of adjacency matrix
	 * @return the String
	 */
	public String adjacencyString() {
		int dim = adjMatrix.length;
		String s = "Adjacency matrix";
		s += " (size " + dim + "x" + dim + " = " + dim* dim + " integers):";
		for (int i = 0; i < dim; i ++) {
			s += "\n\t"+i+": ";
			for (int j = 0; j < adjMatrix[i].length; j++) {
			s += adjMatrix[i][j] + ", ";
			}
		}
		return s;
	}

	public String getSquaredAdjacencyString(){
		int[][] newAdjMatrix = matrixMul(adjMatrix,adjMatrix);
		int dim = newAdjMatrix.length;
		String s = "Adjacency matrix";
		s += " (size " + dim + "x" + dim + " = " + dim* dim + " integers):";
		for (int i = 0; i < dim; i ++) {
			s += "\n\t"+i+": ";
			for (int j = 0; j < newAdjMatrix[i].length; j++) {
				s += newAdjMatrix[i][j] + ", ";
			}
		}
		return s;
	}

	public static void main (String[] args) {
		GraphAdjMatrix gradm = new GraphAdjMatrix();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", gradm);
		System.out.println();
		System.out.println(gradm);

		System.out.println("Double hops from 1: "+gradm.getDistance2(1));

		System.out.println("Squared matrix: "+gradm.getSquaredAdjacencyString());
	}

}
