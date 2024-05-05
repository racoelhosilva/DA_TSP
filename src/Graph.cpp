#include "Graph.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;

Graph::Graph() = default;

Graph::~Graph() {
    for (Vertex* v: vertexSet_)
        delete v;
}

Vertex *Graph::findVertex(int id) const {
    if (id < 0 || id >= (int)vertexSet_.size())
        return nullptr;
    return vertexSet_[id];
}

bool Graph::addVertex(Vertex *v) {
    if (v->getId() != (int)vertexSet_.size())
        return false;
    vertexSet_.push_back(v);
    return true;
}

bool Graph::addEdge(int src, int dest, double w) const {
    Vertex *u = findVertex(src), *v = findVertex(dest);
    if (u == nullptr || v == nullptr)
        return false;
    Edge *e = u->addEdge(v, w), *er = v->addEdge(u, w);
    e->setReverse(er);
    er->setReverse(e);
    return true;
}

std::vector<Vertex*> Graph::getVertexSet() const {
    return vertexSet_;
}

int Graph::getNumEdges() const {
    int numDirEdges = 0;
    for (Vertex *vertex: vertexSet_)
        numDirEdges += (int)vertex->getAdj().size();
    return numDirEdges / 2;
}

void backtrackingTspAux(Vertex* curr, int verticesLeft, double currDist, double &minDist) {
    if (verticesLeft == 0) {
        Edge *pathToStart = curr->getPathToStart();
        if (pathToStart != nullptr && currDist + pathToStart->getWeight() < minDist) {
            minDist = currDist + pathToStart->getWeight();
            curr->setPath(nullptr);
        }
        return;
    }

    double newMinDist = minDist;
    curr->setVisited(true);

    for (Edge *edge: curr->getAdj()) {
        if (currDist + edge->getWeight() >= minDist)
            continue;

        Vertex *dest = edge->getDest();
        if (dest->isVisited())
            continue;

        backtrackingTspAux(dest, verticesLeft - 1, currDist + edge->getWeight(), newMinDist);
        if (newMinDist < minDist) {
            minDist = newMinDist;
            curr->setPath(edge);
        }
    }

    curr->setVisited(false);
}

double Graph::backtrackingTsp(int startId) {
    Vertex *start = findVertex(startId);
    if (start == nullptr)
        return -1;

    for (Vertex *vertex: vertexSet_) {
        vertex->setVisited(false);
        vertex->setPathToStart(nullptr);
    }

    start->setVisited(true);
    for (Edge *edge: start->getAdj()) {
        Vertex *dest = edge->getDest();
        dest->setPathToStart(edge->getReverse());
    }

    double minDist = std::numeric_limits<double>::infinity();
    backtrackingTspAux(start, (int)vertexSet_.size() - 1, 0, minDist);
    return minDist;
}

double Graph::heldKarpTsp(int startId) {
    return 0.0;
}

double Graph::doubleMstTsp(int startId) {
    return 0.0;
}

double Graph::nearestNeighbourTsp(int startId) {
    return 0.0;
}

double Graph::christofidesTsp(int startId) {
    return 0.0;
}

double Graph::realWorldTsp(int startId) {
    return 0.0;
}

Graph * Graph::parse(const std::string& edgeFilename, const std::string& nodeFilename){
    return new Graph;
}

Graph *Graph::parseToyGraph(const std::string& edgeFilename) {
    ifstream edgeFile(edgeFilename);
    if (edgeFile.fail())
        return nullptr;

    int numVertices = 0, id1, id2;
    double dist;
    char comma1, comma2;
    string line;
    istringstream iss;
    getline(edgeFile, line); // Ignore first line
    if (edgeFile.fail())
        return nullptr;
    while (getline(edgeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> id2;
        if (iss.fail() || comma1 != ',')
            return nullptr;
        numVertices = max(max(numVertices, id1 + 1), id2 + 1);
    }

    auto graph = new Graph;
    for (int id = 0; id < numVertices; id++)
        graph->addVertex(new Vertex(id));

    edgeFile.clear();
    edgeFile.seekg(0);
    getline(edgeFile, line);

    while (getline(edgeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> id2 >> comma2 >> dist;
        if (iss.fail() || comma1 != ',' || comma2 != ',' || !graph->addEdge(id1, id2, dist)) {
            delete graph;
            return nullptr;
        }
    }

    return graph;
}

Graph *Graph::parseMediumGraph(const std::string &nodeFilename, const std::string &edgeFilename) {
    ifstream nodeFile(nodeFilename), edgeFile(edgeFilename);
    if (nodeFile.fail() || edgeFile.fail())
        return nullptr;

    istringstream iss(edgeFilename);
    string text;
    int numVertices;

    getline(iss, text, '_');
    getline(iss, text, '.');
    numVertices = stoi(text);

    auto graph = new Graph;

    string line;
    int id1, id2;
    double lat, lon, dist;
    char comma1, comma2;

    getline(nodeFile, line);
    if (nodeFile.fail())  {
        delete graph;
        return nullptr;
    }
    
    while (getline(nodeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> lat >> comma2 >> lon;
        if (iss.fail() || comma1 != ',' || comma2 != ',') {
            delete graph;
            return nullptr;
        }
        if (id1 >= numVertices)
            break;
        if (!graph->addVertex(new Vertex(id1, lat, lon))) {
            delete graph;
            return nullptr;
        }
    }

    while (getline(edgeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> id2 >> comma2 >> dist;
        if (iss.fail() || comma1 != ',' || comma2 != ',' || !graph->addEdge(id1, id2, dist)) {
            delete graph;
            return nullptr;
        }
    }

    return graph;
}

Graph *Graph::parseRealWorldGraph(const std::string &nodeFilename, const std::string &edgeFilename) {
    ifstream nodeFile(nodeFilename), edgeFile(edgeFilename);
    if (nodeFile.fail() || edgeFile.fail())
        return nullptr;

    auto graph = new Graph;

    string line;
    int id1, id2;
    double lat, lon, dist;
    char comma1, comma2;
    istringstream iss;

    getline(nodeFile, line);
    if (nodeFile.fail())  {
        delete graph;
        return nullptr;
    }
    while (getline(nodeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> lat >> comma2 >> lon;
        if (iss.fail() || comma1 != ',' || comma2 != ',' || !graph->addVertex(new Vertex(id1, lat, lon))) {
            delete graph;
            return nullptr;
        }
    }

    getline(edgeFile, line);
    if (edgeFile.fail())  {
        delete graph;
        return nullptr;
    }
    while (getline(edgeFile, line)) {
        if (line.empty())
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> id2 >> comma2 >> dist;
        if (iss.fail() || comma1 != ',' || comma2 != ',' || !graph->addEdge(id1, id2, dist)) {
            delete graph;
            return nullptr;
        }
    }

    return graph;
}

double Graph::haversineDistance(const Vertex *v1, const Vertex *v2) {
    double dLat = (v2->getLatitude() - v1->getLatitude()) * M_PI / 180.0;
    double dLon = (v2->getLongitude() - v1->getLongitude()) * M_PI / 180.0;

    // convert to radians
    double lat1 = (v1->getLatitude()) * M_PI / 180.0;
    double lat2 = (v2->getLatitude()) * M_PI / 180.0;

    // apply formula
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double earthRadius = 6371;
    double c = 2 * asin(sqrt(a));

    return earthRadius * c;
}
