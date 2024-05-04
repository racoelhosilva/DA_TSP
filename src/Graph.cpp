#include "Graph.h"

#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

Graph::Graph() = default;

Graph::~Graph() {
    for (Vertex* v: vertexSet)
        delete v;
}

Vertex *Graph::findVertex(int id) const {
    if (id < 0 || id >= (int)vertexSet.size())
        return nullptr;
    return vertexSet[id];
}

bool Graph::addVertex(Vertex *v) {
    if (v->getId() != (int)vertexSet.size())
        return false;
    vertexSet.push_back(v);
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
    return vertexSet;
}

int Graph::getNumEdges() const {
    int numDirEdges = 0;
    for (Vertex *vertex: vertexSet)
        numDirEdges += (int)vertex->getAdj().size();
    return numDirEdges / 2;
}

double Graph::backtrackingTsp() {
    return 0.0;
}

double Graph::heldKarpTsp() {
    return 0.0;
}

double Graph::doubleMstTsp() {
    return 0.0;
}

double Graph::nearestNeighbourTsp() {
    return 0.0;
}

double Graph::christofidesTsp() {
    return 0.0;
}

double Graph::realWorldTsp() {
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
        if (line == "")
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
        if (line == "")
            break;
        iss.clear();
        iss.str(line);
        iss >> id1 >> comma1 >> id2 >> comma2 >> dist;
        if (iss.fail() || comma1 != ',' || comma2 != ',') {
            delete graph;
            return nullptr;
        }
        graph->addEdge(id1, id2, dist);
    }

    return graph;
}
