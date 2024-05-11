#ifndef DA_TSP_VERTEX_H
#define DA_TSP_VERTEX_H

#include "Edge.h"
#include <vector>
#include <limits>

class Edge;

/**
 * @brief Class that represents a vertex
 * @tparam T Type of the vertex's info
 */
class Vertex {
public:
    /**
     * @brief Constructor of the Vertex class
     * @param id The information of the vertex
     */
    explicit Vertex(int id, double latitude = std::numeric_limits<double>::quiet_NaN(),
                    double longitude = std::numeric_limits<double>::quiet_NaN());

    /**
     * @brief Destructor of the Vertex class
     * @details Complexity: O(E), where E is the number of outgoing edges of the vertex
     */
    virtual ~Vertex();

    /**
     * @brief Returns the info of the vertex
     * @return The info of the vertex
     */
    int getId() const;

    /**
     * @brief Returns the latitude of the vertex
     * @return Latitude of the vertex
     */
    double getLatitude() const;

    /**
     * @brief Returns the longitude of the vertex
     * @return Longitude of the vertex
     */
    double getLongitude() const;

    /**
     * @brief Returns the outgoing edges of the vertex
     * @return Vector of references to the vertex's outgoing edges
     */
    std::vector<Edge *> getAdj() const;

    /**
     * @brief Returns if the vertex was visited
     * @return True if the vertex was set as visited, and false otherwise
     */
    bool isVisited() const;

    /**
     * @brief Returns if the vertex is being processed
     * @return True if the vertex was set as processed, and false otherwise
     */
    bool isProcessing() const;

    /**
     * @brief Returns the parent's edge that connects to this vertex (for storing the solution to the TSP)
     * @return The parent's edge to this vertex
     */
    Edge *getPath() const;

    /**
     * @brief Returns the edge that connects this vertex to the starting vertex
     * @return The edge from this vertex to the start
     */
    Edge *getPathToStart() const;

    /**
     * @brief Returns the degree of the vertex
     * @return Degree of the vertex
     */
    int getDegree() const;

    /**
     * @brief Returns the edge that connects this vertex to the destination vertex
     * Complexity: O(E), where E is the number of outgoing edges of the vertex
     * @param destId The destination vertex's id
     * @return The edge that connects this vertex to the destination vertex, or nullptr if it does not exist
     */
    Edge *findEdgeTo(int destId) const;

    /**
     * @brief Sets if the vertex has been visited
     * @param visited Whether the vertex was visited or not
     */
    void setVisited(bool visited);

    /**
     * @brief Sets if the vertex is being processed
     * @param processing Whether the vertex is being processed or not
     */
    void setProcessing(bool processing);

    /**
     * @brief Sets the parent's edge that connects to this vertex (for storing the solution to the TSP)
     * @param path The parent's edge to this vertex
     */
    void setPath(Edge *path);

    /**
     * @brief Sets the edge that connects this vertex to the starting vertex
     * @param pathToStart Edge from this vertex to the start
     */
    void setPathToStart(Edge *pathToStart);

    /**
     * @brief Sets the degree of the vertex
     * @param degree Degree of the vertex
     */
    void setDegree(int degree);

    /**
     * @brief Adds an outgoing edge from this vertex to dest, and the same edge as an incoming edge of dest
     * Complexity: O(1).
     * @param dest Destination of the pipe
     * @param w Edge's weight
     * @return Pointer to the newly created edge
     */
    Edge *addEdge(Vertex *dest, double w);

protected:
    int id_;
    double latitude_, longitude_;

    std::vector<Edge *> adj_;
    bool visited_ = false;
    bool processing_ = false;
    Edge *path_ = nullptr;
    Edge *pathToStart_ = nullptr;
    int degree_ = 0;
};

#endif //DA_TSP_VERTEX_H
