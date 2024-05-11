#ifndef DA_TSP_GRAPH_H
#define DA_TSP_GRAPH_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include "Vertex.h"

/**
 * @brief Class that represents an undirected graph, whose vertices have continuous integer ids, starting from 0
 */
class Graph {
public:
    /**
     * @brief Constructor of the Graph class
     */
    Graph();

    /**
     * @brief Destructor of the Graph class
     * @details Complexity: O(V+E), where V is the number of vertices and E the number of edges in the graph.
     */
    virtual ~Graph();

    /**
     * @brief Finds a vertex in the graph
     * @details Complexity: O(1).
     * @param in Info of the vertex to search
     * @return Reference to the vertex wanted, or nullptr of not found
     */
    Vertex *findVertex(int id) const;

    /**
     * @brief Adds a vertex to the graph
     * @details The vertex must be inserted in a way to keep a sequential order of ids, i.e. its id must be one more
     * than the id of the last inserted vertex. Complexity: O(1).
     * @param v Reference to the vertex to remove
     * @return True if the vertex was successfully added, and false if it already exists
     */
    bool addVertex(Vertex* v);

    /**
     * @brief Adds an undirected edge from the vertex with info src to the vertex with info dest
     * @details The undirected edge is represented by two directed edges, one from the source to the destination and
     * the other in the opposite direction. Complexity: O(1).
     * @param src Info of the source vertex
     * @param dest Info of the destination vertex
     * @param w Weight of the edge
     * @return True if the edge was successfully added, and false otherwise (if one of the vertices were not found)
     */
    bool addEdge(int src, int dest, double w) const;

    /**
     * @brief Returns a set with all the vertices of the graph.
     * @details Complexity: O(1).
     * @return Set with the graph's vertices
     */
    std::vector<Vertex*> getVertexSet() const;

    /**
     * @brief Calculates the number of (undirected) edges in the graph
     * @details Complexity: O(V), where V is the number of vertices in the graph.
     * @return Number of edges in the graph
     */
    int getNumEdges() const;

    /**
     * @brief Calculates the exact solution of the TSP using a backtracking approach with branch-and-bound.
     * @details Complexity: O(V!), where V is the number of vertices in the graph.
     * @param startId The id of the vertex to start the TSP
     * @return The cost of the solution found
     */
    double backtrackingTsp();


    double heldKarpTsp();
    double doubleMstTsp(int startId);
    double nearestNeighbourTsp(int startId);
    double christofidesTsp(int startId);
    double realWorldTsp(int startId);

    /**
     * @brief Parses a graph from the file with the edges
     * @details Complexity: O(E), where E the number of edges in the file.
     * @param edgeFilename The name of the file with the edges of the graph
     * @return Pointer to the graph parsed from the file
     */
    static Graph *parseToyGraph(const std::string &edgeFilename);

    /**
     * @brief Parses an extra fully connected graph from the file with the nodes and the file with the edges
     * @details The edge file name must indicate the number of vertices the graph must have. Complexity: O(V+E), where V
     * is the number of nodes and E the number of edges.
     * @param nodeFilename Filename of the node file
     * @param edgeFilename Filename of the edge file
     * @return Pointer to the graph parsed from the files
     */
    static Graph *parseMediumGraph(const std::string &nodeFilename, const std::string &edgeFilename);

    /**
     * @brief Parses a real world graph from the file with the nodes and the file with the edges
     * @details In order to be parsed correctly, both files must be in the csv format with a header line. The node file
     * must also enumerate the vertices in increasing order of id, starting from 0.
     * Complexity: O(V+E), where V is the number of nodes and E the number of edges.
     * @param nodeFilename Filename of the node file
     * @param edgeFilename Filename of the edge file
     * @return Pointer to the graph parsed from the files
     */
    static Graph *parseRealWorldGraph(const std::string &nodeFilename, const std::string &edgeFilename);

private:
    static double haversineDistance(const Vertex *v1, const Vertex *v2);
    double **getDistMatrix() const;
    double **getCompleteDistMatrix() const;

    template<class T>
    void deleteMatrix(T **matrix) const;

    Graph *createCompleteCopy() const;

    bool respectsTriangularInequality();

    void kruskalDfs(Vertex *vertex);
    void kruskal(std::vector<Edge*> &edges);
    void minWeightPerfectMatchingGreedy(const std::vector<Edge*> &sortedEdges);
    double hamiltonianCircuitDfs(Vertex *vertex, Vertex *&last);

    std::vector<Vertex*> vertexSet_;
};

#endif //DA_TSP_GRAPH_H
