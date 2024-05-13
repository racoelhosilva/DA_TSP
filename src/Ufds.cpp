// Implementation based on the files provided in the practical classes

#include "Ufds.h"

Ufds::Ufds(int n) {
    path_.resize(n);
    rank_.resize(n);
    for (int i = 0; i < n; i++) {
        path_[i] = i;
        rank_[i] = 0;
    }
}

int Ufds::findSet(int i) {
    if (path_[i] != i) path_[i] = findSet(path_[i]);
    return path_[i];
}

bool Ufds::isSameSet(int i, int j) {
    return findSet(i) == findSet(j);
}

void Ufds::linkSets(int i, int j) {
    int x = findSet(i), y = findSet(j);
    if (x != y) {
        if (rank_[x] > rank_[y])
            path_[y] = x;
        else
            path_[x] = y;
        if (rank_[x] == rank_[y])
            rank_[y]++;
    }
}




