#ifndef DA_TSP_UFDS_H
#define DA_TSP_UFDS_H

#include <vector>

class Ufds {
public:
    /**
     * @brief Constructor of the Ufds class
     * @details Complexity: O(n)
     * @param n The number of elements in the Ufds
     */
    Ufds(int n);

    /**
     * @brief Finds the set where the item i belongs
     * @details Complexity: O(α(n)) amortized, where α(n) is the inverse Ackermann function
     * @param i Item to check
     * @return The item's set number
     */
    int findSet(int i);

    /**
     * @brief Checks if two items are part of the same set
     * @details Complexity: O(α(n)) amortized, where α(n) is the inverse Ackermann function
     * @param i First item
     * @param j Second item
     * @return True if the items are part of the same set, and false otherwise
     */
    bool isSameSet(int i, int j);

    /**
     * @brief Links the two sets where the items are stored
     * @details Complexity: O(α(n)) amortized, where α(n) is the inverse Ackermann function
     * @param i First item
     * @param j Second item
     */
    void linkSets(int i, int j);

private:
    std::vector<int> path_; // Ancestor of node i (which can be itself). It is used to determine if two nodes are part of the same set.
    std::vector<int> rank_; // Upper bound for the height of a tree whose root is node i.
};


#endif //DA_TSP_UFDS_H
