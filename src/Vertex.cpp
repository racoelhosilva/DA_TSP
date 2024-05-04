#include "Vertex.h"

#include <limits>

Vertex::Vertex(int id, double latitude, double longitude)
               : id(id), latitude(latitude), longitude(longitude) {}

Vertex::~Vertex() {
    for (Edge* edge: adj)
        deleteEdge(edge);
    adj.clear();
};

Edge * Vertex::addEdge(Vertex *d, double w) {
    auto newEdge = new Edge(this, d, w);
    adj.push_back(newEdge);
    return newEdge;
}

int Vertex::getId() const {
    return id;
}

double Vertex::getLatitude() const {
    return latitude;
}

double Vertex::getLongitude() const {
    return longitude;
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

    auto it = dest->adj.begin();
    while (it != dest->adj.end()) {
        if ((*it)->getOrig()->getId() == id) {
            it = dest->adj.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}
