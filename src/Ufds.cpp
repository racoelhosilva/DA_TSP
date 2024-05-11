// Implementation based on the files provided in the practical classes

#include "Ufds.h"

Ufds::Ufds(int n) {
    path.resize(n);
    rank.resize(n);
    for (int i = 0; i < n; i++) {
        path[i] = i;
        rank[i] = 0;
    }
}

int Ufds::findSet(int i) {
    if (path[i] != i) path[i] = findSet(path[i]);
    return path[i];
}

bool Ufds::isSameSet(int i, int j) {
    return findSet(i) == findSet(j);
}

void Ufds::linkSets(int i, int j) {
    int x = findSet(i), y = findSet(j);
    if (x != y) {
        if (rank[x] > rank[y])
            path[y] = x;
        else
            path[x] = y;
        if (rank[x] == rank[y])
            rank[y]++;
    }
}




