#include "Vertex.h"

#include <limits>

Vertex::Vertex(int id, double latitude, double longitude)
               : id_(id), latitude_(latitude), longitude_(longitude) {}

Vertex::~Vertex() {
    for (Edge* edge: adj_)
        delete edge;
    adj_.clear();
};

Edge * Vertex::addEdge(Vertex *d, double w) {
    auto newEdge = new Edge(this, d, w);
    adj_.push_back(newEdge);
    return newEdge;
}

int Vertex::getId() const {
    return id_;
}

double Vertex::getLatitude() const {
    return latitude_;
}

double Vertex::getLongitude() const {
    return longitude_;
}

std::vector<Edge*> Vertex::getAdj() const {
    return adj_;
}

bool Vertex::isVisited() const {
    return visited_;
}


bool Vertex::isProcessing() const {
    return processing_;
}

Edge *Vertex::getPath() const {
    return path_;
}

Edge *Vertex::getPathToStart() const {
    return pathToStart_;
}

void Vertex::setVisited(bool visited) {
    visited_ = visited;
}

void Vertex::setProcessing(bool processing) {
    processing_ = processing;
}

void Vertex::setPath(Edge *path) {
    path_ = path;
}

void Vertex::setPathToStart(Edge *pathToStart) {
    pathToStart_ = pathToStart;
}

int Vertex::getDegree() const {
    return degree_;
}

void Vertex::setDegree(int degree) {
    Vertex::degree_ = degree;
}

Edge *Vertex::findEdgeTo(int destId) const {
    for (Edge* edge: adj_) {
        if (edge->getDest()->getId() == destId)
            return edge;
    }
    return nullptr;
}
