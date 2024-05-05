#ifndef DA_TSP_GRAPH_H
#define DA_TSP_GRAPH_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include "Vertex.h"

/**
 * @brief Class that represents a generic graph
 * @tparam T Type of the vertices' information
 */
class Graph {
public:
    /**
     * @brief Constructor of the Graph class
     */
    Graph();

    /**
     * @brief Destructor of the Graph class
     * @details Complexity: O(V*E), where V is the number of vertices and E the number of edges in the graph.
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
     * @brief Adds a vertex to the graph if it does not exist in the graph
     * @details Complexity: O(1).
     * @param v Reference to the vertex to remove
     * @return True if the vertex was successfully added, and false if it already exists
     */
    bool addVertex(Vertex* v);

    /**
     * @brief Adds an edge from the vertex with info src to the vertex with info dest
     * @details Complexity: O(1).
     * @param src Info of the source vertex
     * @param dest Info of the destination vertex
     * @param w Weight of the edge
     * @return True if the edge was successfully added, and false otherwise (if one of the vertices were not found)
     */
    bool addEdge(int src, int dest, double w) const;

    /**
     * @brief Returns a set with all the vertices of the graph
     * @return Set with the graph's vertices
     */
    std::vector<Vertex*> getVertexSet() const;

    int getNumEdges() const;

    double backtrackingTsp(int startId);
    double heldKarpTsp(int startId);
    double doubleMstTsp(int startId);
    double nearestNeighbourTsp(int startId);
    double christofidesTsp(int startId);
    double realWorldTsp(int startId);

    static Graph * parse(const std::string &edgeFilename = "", const std::string &nodeFilename = "");

    static Graph *parseToyGraph(const std::string &edgeFilename);
    static Graph *parseMediumGraph(const std::string &nodeFilename, const std::string &edgeFilename);
    static Graph *parseRealWorldGraph(const std::string &nodeFilename, const std::string &edgeFilename);

private:
    static double haversineDistance(const Vertex *v1, const Vertex *v2);
    double **getDistMatrix();
    double **getCompleteDistMatrix();
    void deleteMatrix(double **matrix);

    bool respectsTriangularInequality();

    void kruskal();

    std::vector<Vertex*> vertexSet_;
};

#endif //DA_TSP_GRAPH_H
