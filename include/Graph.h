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
    /**
     * @brief Calculates the distance between two vertices using the haversine distance formula
     * @param v1 The first vertex
     * @param v2 The second vertex
     * @details Complexity: O(1) (assuming the functions std::sin, std::cos, std::sqrt and std::atan2 are O(1)).
     * @return Haversine distance between the two vertices
     */
    static double haversineDistance(const Vertex *v1, const Vertex *v2);

    /**
     * @brief Returns the matrix representation of the graph
     * @details The matrix returned is a |V| x |V| matrix (w_ij), where w_ij is the weight of the edge that goes from
     * the vertex with index i to the vertex with index j. If there is no edge between the vertices, w_ij is set to
     * infinity. The matrix is stored as a dynamically allocated 2D double array, so it must be freed (e.g. using
     * deleteMatrix) after being used. Complexity: O(V^2), where V is the number of vertices in the graph.
     * @return Matrix representation of the graph
     */
    double **getDistMatrix() const;

    /**
     * @brief Returns the complete matrix representation of the graph
     * @details The matrix returned is a |V| x |V| matrix (w_ij), where w_ij is the weight of the edge that goes from
     * the vertex with index i to the vertex with index j. If there is no edge between the vertices, w_ij is set to
     * the haversine distance between the two vertices. The matrix is stored as a dynamically allocated 2D double array,
     * so it must be freed (e.g. using deleteMatrix) after being used. Complexity: O(V^2), where V is the number of
     * vertices in the graph.
     * @return Complete matrix representation of the graph
     */
    double **getCompleteDistMatrix() const;

    /**
     * @brief Frees a dynamically allocated 2D square matrix, with side n
     * @tparam T Type of the matrix elements
     * @param matrix Matrix to be freed from memory
     * @param n Side of the square matrix
     * @details Complexity: O(n^2).
     */
    template<class T>
    static void deleteMatrix(T **matrix, int n);

    /**
     * @brief Creates a fully connected copy of the graph
     * @details If there is no edge between two vertices, one edge with the haversine distance as the weight is added.
     * Complexity: O(V^2), where V is the number of vertices in the graph.
     * @return Complete copy of the graph
     */
    Graph *createCompleteCopy() const;

    /**
     * @brief Checks if all the edges in the graph respect the triangular inequality
     * @details The triangular inequality states that, for any three vertices u, v and w, c(u, w) <= c(u, v) + c(v, w).
     * Complexity: O(V^3), where V is the number of vertices in the graph.
     * @return True if all the edges respect the triangular inequality, and false otherwise
     */
    bool respectsTriangularInequality();

    /**
     * @brief Auxiliary function to the Kruskal algorithm, performing a depth-first search through the selected edges,
     * from a specified vertex, to find a minimum spanning tree and updating the graph accordingly
     * @details Complexity: O(V), where V is the number of vertices (the number of selected undirected edges in the
     * kruskal will always be V-1).
     * @param vertex The vertex to start the depth-first search
     */
    void kruskalDfs(Vertex *vertex);

    /**
     * @brief Performs the Kruskal algorithm to find the minimum spanning tree of a connected graph
     * @details It also sorts the vector of edges received in ascending order of weight. The MST is recorded in the path
     * edges of the vertices. Complexity: O(E log V), where E is the number of edges in the graph and V is the number of
     * vertices.
     * @param edges Vector with the edges of the graph
     */
    void kruskal(std::vector<Edge*> &edges);

    /**
     * @brief Calculates an approximated minimum weight perfect matching of the unvisited vertices, using a greedy
     * heuristic
     * @details This approach chooses the smallest edge available at each step, until no more edges that match two
     * unmatched vertices are left. The edges of the matching are set as selected. Complexity: O(E), where E is the
     * number of edges.
     * @param sortedEdges Vector of edges, sorted in ascending order of weight
     */
    void minWeightPerfectMatchingGreedy(const std::vector<Edge*> &sortedEdges);

    /**
     * @brief Performs a DFS to find a hamiltonian circuit of a complete graph, by connecting the vertices in pre-order.
     * @details The DFS is performed using only the selected edges of the graph. Complexity: O(V+E), where V is the
     * number of vertices in the graph and E is the number of edges. In the case of the call in the christofidesTsp, the
     * complexity is O(V), as the number of selected edges will be, at most, 3V/2.
     * @param vertex The vertex to start the DFS
     * @param last The last vertex visited in the DFS
     * @return The cost of the hamiltonian circuit found
     */
    double hamiltonianCircuitDfs(Vertex *vertex, Vertex *&last);

    std::vector<Vertex*> vertexSet_;
};

#endif //DA_TSP_GRAPH_H
