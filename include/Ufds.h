#ifndef DA_TSP_UFDS_H
#define DA_TSP_UFDS_H


#include <vector>

class Ufds {
public:
    Ufds(int n);
    int findSet(int i);
    bool isSameSet(int i, int j);
    void linkSets(int i, int j);

private:
    std::vector<int> path; // Ancestor of node i (which can be itself). It is used to determine if two nodes are part of the same set.
    std::vector<int> rank; // Upper bound for the height of a tree whose root is node i.
};


#endif //DA_TSP_UFDS_H
