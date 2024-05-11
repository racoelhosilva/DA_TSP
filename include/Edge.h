#ifndef DA_TSP_EDGE_H
#define DA_TSP_EDGE_H


#include "Vertex.h"

class Vertex;

/**
 * @brief Class that represents an edge in a graph
 * @tparam T Type of the info of the vertices that the edge connects
 */
class Edge {
public:
    /**
     * @brief Constructor of the Edge class
     * @param orig Pointer to the origin vertex of the edge
     * @param dest Pointer to the destination vertex of the edge
     * @param weight The weight of the edge
     */
    Edge(Vertex *orig, Vertex *dest, double weight);

    /**
     * @brief Destructor of the Edge class
     * @details Complexity: O(1)
     */
    virtual ~Edge();

    /**
     * @brief Returns destination vertex of the edge
     * @return Pointer to the destination vertex
     */
    Vertex *getDest() const;

    /**
     * @brief Returns the origin vertex of the edge
     * @return Pointer to the origin vertex
     */
    Vertex *getOrig() const;

    /**
     * @brief Returns the weight of the edge
     * @return The weight of the edge
     */
    double getWeight() const;

    /**
     * @brief Returns if the edge is selected
     * @return True if the edge was set as selected, and false otherwise
     */
    bool isSelected() const;

    /**
     * @brief Returns the reverse edge
     * @return Pointer to the reverse edge
     */
    Edge *getReverse() const;

    /**
     * @brief Sets if the edge is selected
     * @details If the edge has a reverse edge, it also sets the reverse edge as selected or unselected
     * @param selected True to set the edge as selected, false to set it as unselected
     */
    void setSelected(bool selected);

    /**
     * @brief Sets the reverse edge
     * @param reverse Pointer to the reverse edge
     */
    void setReverse(Edge *reverse);

private:
    Vertex* orig_;
    Vertex* dest_;
    double weight_;
    bool selected_;
    Edge* reverse_;
};

#endif //DA_TSP_EDGE_H
