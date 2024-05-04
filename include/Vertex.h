#ifndef DA_TSP_VERTEX_H
#define DA_TSP_VERTEX_H

#include "Edge.h"
#include <vector>

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
    explicit Vertex(int id);

    /**
     * @brief Destructor of the Vertex class
     */
    virtual ~Vertex();

    /**
     * @brief Returns the info of the vertex
     * @return The info of the vertex
     */
    int getId() const;

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
     * @brief Returns the parent's edge that connects to this vertex (auxiliary to graph searches)
     * @return The parent's edge to this vertex
     */
    Edge *getPath() const;

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
     * @brief Sets the parent's edge that connects to this vertex (auxiliary to graph searches)
     * @param path The parent's edge to this vertex
     */
    void setPath(Edge *path);

    /**
     * @brief Adds an outgoing edge from this vertex to dest, and the same edge as an incoming edge of dest
     * Complexity: O(1).
     * @param dest Destination of the pipe
     * @param w Edge's weight
     * @return Pointer to the newly created edge
     */
    virtual Edge *addEdge(Vertex *dest, double w);

protected:
    int id;
    double latitude, longitude;

    std::vector<Edge *> adj;
    bool visited = false;
    bool processing = false;
    Edge *path = nullptr;

    /**
     * @brief Removes the edge from the list of incoming edges of the destination and frees it.
     * Complexity: O(E), where E is the number of incoming edges of the destination vertex
     * @param edge The edge to be deleted
     */
    void deleteEdge(Edge *edge);
};

Vertex::Vertex(int id): id(id) {}

Vertex::~Vertex() {
    for (Edge* edge: adj)
        deleteEdge(edge);
    return;
};

Edge * Vertex::addEdge(Vertex *d, double w) {
    auto newEdge = new Edge(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

bool Vertex::removeEdge(int in) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge *edge = *it;
        Vertex *dest = edge->getDest();
        if (dest->getId() == in) {
            it = adj.erase(it);
            deleteEdge(edge);
            removedEdge = true;
        } else {
            it++;
        }
    }
    return removedEdge;
}

void Vertex::removeOutgoingEdges() {
    for(Edge *edge: this->adj) {
        deleteEdge(edge);
    }
    adj.clear();
}

int Vertex::getId() const {
    return this->id;
}

std::vector<Edge*> Vertex::getAdj() const {
    return this->adj;
}

bool Vertex::isVisited() const {
    return this->visited;
}


bool Vertex::isProcessing() const {
    return this->processing;
}

Edge *Vertex::getPath() const {
    return this->path;
}

std::vector<Edge *> Vertex::getIncoming() const {
    return this->incoming;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

void Vertex::setProcessing(bool processing) {
    this->processing = processing;
}

void Vertex::setPath(Edge *path) {
    this->path = path;
}

void Vertex::deleteEdge(Edge *edge) {
    Vertex *dest = edge->getDest();

    auto it = dest->incoming.begin();
    while (it != dest->incoming.end()) {
        if ((*it)->getOrig()->getId() == id) {
            it = dest->incoming.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}

#endif //DA_TSP_VERTEX_H
