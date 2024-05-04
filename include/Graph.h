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

    double backtrackingTSP();
    double heldKarpTSP();
    double doubleMSTTSP();
    double nearestNeighbourTSP();
    double christofidesTSP();
    double realWorldTSP();

    void parseEdges(std::string edgeFilename);

private:
    std::vector<Vertex*> vertexSet;
};

Graph::Graph() = default;

Graph::~Graph() {
    for (Vertex* v: vertexSet)
        delete v;
}

Vertex *Graph::findVertex(int id) const {
    if (id < 0 || id >= vertexSet.size())
        return nullptr;
    return vertexSet[id];
}

bool Graph::addVertex(Vertex *v) {
    if (v->getId() != vertexSet.size())
        return false;
    vertexSet[v->getId()] = v;
    return true;
}

bool Graph::addEdge(int src, int dest, double w) const {
    Vertex *u = findVertex(src), *v = findVertex(dest);
    if (u == nullptr || v == nullptr)
        return false;
    Edge *e = u->addEdge(v, w), *er = v->addEdge(u, w);
    e->setReverse(er);
    er->setReverse(e);
    return true;
}

std::vector<Vertex*> Graph::getVertexSet() const {
    return vertexSet;
}

bool ;

#endif //DA_TSP_GRAPH_H
