#include "Graph.h"
#include "Ufds.h"

#include <fstream>
#include <sstream>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <algorithm>

#define TRAIL_ZERO(n) __builtin_ctz(n)

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
        vertex->setPath(nullptr);
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

uint64_t initMask(int k) {
    return ((uint64_t)1 << k) - 1;
}

uint64_t nextMask(uint64_t mask, int k) {
    uint64_t aux = mask | (mask - 1);
    return (aux + 1) | (((~aux & (aux + 1)) - 1) >> (TRAIL_ZERO(mask) + 1));
}

uint64_t subsetMask(uint64_t mask, int v) {
    return (mask & (((uint64_t)1 << v) - 1)) | ((mask & (UINT64_MAX << (v + 1))) >> 1);
}

double Graph::heldKarpTsp(int startId) {
    int numVertex = (int)vertexSet_.size();
    if (numVertex <= 1)
        return 0;
    if (numVertex > 64)
        return -1;  // Because we're using a 64-bit bitmask

    auto dist = getDistMatrix();

    const uint64_t numSubsets = (uint64_t)1 << (numVertex - 2);
    auto dp = new double*[numVertex - 1];
    for (int i = 0; i < numVertex - 1; i++) {
        dp[i] = new double[numSubsets];
        for (uint64_t j = 0; j < numSubsets; j++)
            dp[i][j] = numeric_limits<double>::infinity();
    }

    const uint64_t maxMask = ((uint64_t)1 << (numVertex - 1)) - 1;
    uint64_t iMask, jMask;
    for (int i = 1; i < numVertex; i++)
        dp[i - 1][0] = dist[0][i];
    for (int k = 2; k < numVertex; k++) {
        for (uint64_t mask = initMask(k); mask <= maxMask; mask = nextMask(mask, k)) {
            int i = 0;
            for (uint64_t sel1 = mask; sel1 != 0; sel1 >>= (TRAIL_ZERO(sel1) + 1)) {
                i += TRAIL_ZERO(sel1) + 1;
                iMask = subsetMask(mask, i - 1);
                int j = 0;
                for (uint64_t sel2 = mask; sel2 != 0; sel2 >>= (TRAIL_ZERO(sel2) + 1)) {
                    j += TRAIL_ZERO(sel2) + 1;
                    jMask = subsetMask(mask & ~((uint64_t)1 << (i - 1)), j - 1);
                    if (i != j)
                        dp[i - 1][iMask] = min(dp[i - 1][iMask], dp[j - 1][jMask] + dist[j][i]);
                }
            }
        }
    }

    double res = numeric_limits<double>::infinity();
    for (int i = 1; i < numVertex; i++)
        res = min(res, dp[i - 1][initMask(numVertex - 2)] + dist[i][0]);

    for (int i = 0; i < numVertex - 1; i++)
        delete [] dp[i];
    delete [] dp;

    deleteMatrix(dist);

    return res;
}

double Graph::doubleMstTsp(int startId) {
    return 0.0;
}

double Graph::nearestNeighbourTsp(int startId) {
    Vertex *start = findVertex(startId);
    if (start == nullptr)
        return -1;

    for (Vertex *vertex: vertexSet_) {
        vertex->setVisited(false);
        vertex->setPathToStart(nullptr);
    }

    auto dist = getCompleteDistMatrix();

    double distance = 0.0;
    Vertex *cur = start;

    for (int operations = 0; operations < (int)vertexSet_.size()-1; operations++){

        cur->setVisited(true);
        double minDist = std::numeric_limits<double>::infinity();
        Vertex *dest = nullptr;

        for (Vertex *vertex : vertexSet_){
            if (vertex->isVisited()) continue;
            if (dist[vertex->getId()][cur->getId()] >= minDist) continue;

            minDist = dist[cur->getId()][vertex->getId()];
            dest = vertex;
        }

        if (dest == nullptr) {
            return -1;
        }

        cur = dest;
        distance += minDist;
    }
    distance += dist[cur->getId()][startId];

    deleteMatrix(dist);

    return distance;
}

double Graph::christofidesTsp(int startId) {
    if (vertexSet_.empty())
        return 0;

    Graph *copy = createCompleteCopy();

    vector<Edge*> edges;
    edges.reserve(copy->vertexSet_.size() * copy->vertexSet_.size());
    for (Vertex *v: copy->vertexSet_) {
        for (Edge *edge: v->getAdj()) {
            edges.push_back(edge);
            edge->setSelected(false);
        }
    }

    copy->kruskal(edges);
    for (Vertex *vertex: copy->vertexSet_)
        vertex->setVisited(vertex->getDegree() % 2 == 0);
    copy->minWeightPerfectMatchingGreedy(edges);

    for (Vertex *vertex: copy->vertexSet_)
        vertex->setVisited(false);
    Vertex *root = copy->findVertex(0), *last = root;
    double res = copy->hamiltonianCircuitDfs(root, last) + last->findEdgeTo(0)->getWeight();

    delete copy;
    return res;
}

double **Graph::floydWarshall()
{
    double **dist = getDistMatrix();

    for (int k = 0; k < vertexSet_.size(); k++) {
        for (int i = 0; i < vertexSet_.size(); i++) {
            for (int j = 0; j < vertexSet_.size(); j++) {
                if (dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    return dist;
}

Graph *Graph::createAuxGraph(double **dist) const {
    auto newGraph = new Graph;

    for (Vertex *vertex: vertexSet_)
        newGraph->addVertex(new Vertex(vertex->getId(), vertex->getLatitude(), vertex->getLongitude()));

    for (int i = 0; i < vertexSet_.size(); i++) {
        for (int j = i + 1; j < vertexSet_.size(); j++) {
            newGraph->addEdge(i, j, dist[i][j]);
        }
    }

    return newGraph;
}

double Graph::realWorldTsp(int startId) {
    double **dist = floydWarshall();

    auto *auxGraph = createAuxGraph(dist);

    double totalDistance = auxGraph->christofidesTsp(startId);

    deleteMatrix(dist);
    delete auxGraph;

    return totalDistance;
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
    if (isnan(v1->getLatitude()) || isnan(v1->getLongitude())
        || isnan(v2->getLatitude()) || isnan(v2->getLongitude()))
        return numeric_limits<double>::infinity();

    double dLat = (v2->getLatitude() - v1->getLatitude()) * M_PI / 180.0;
    double dLon = (v2->getLongitude() - v1->getLongitude()) * M_PI / 180.0;

    // convert to radians
    double lat1 = (v1->getLatitude()) * M_PI / 180.0;
    double lat2 = (v2->getLatitude()) * M_PI / 180.0;

    // apply formula
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double earthRadius = 6371000;
    double c = 2 * asin(sqrt(a));

    return earthRadius * c;
}

double **Graph::getDistMatrix() const {
    auto matrix = new double*[vertexSet_.size()];
    for (int i = 0; i < (int)vertexSet_.size(); i++) {
        matrix[i] = new double[vertexSet_.size()];
        for (int j = 0; j < (int)vertexSet_.size(); j++)
            matrix[i][j] = i == j ? 0 : numeric_limits<double>::infinity();
    }
    for (Vertex *orig: vertexSet_) {
        for (Edge *edge: orig->getAdj()) {
            Vertex *dest = edge->getDest();
            matrix[orig->getId()][dest->getId()] = edge->getWeight();
        }
    }
    return matrix;
}

double **Graph::getCompleteDistMatrix() const {
    auto matrix = new double*[vertexSet_.size()];
    for (int i = 0; i < (int)vertexSet_.size(); i++) {
        matrix[i] = new double[vertexSet_.size()];
        for (int j = 0; j < (int)vertexSet_.size(); j++)
            matrix[i][j] = i == j ? 0 : numeric_limits<double>::quiet_NaN();
    }
    for (Vertex *orig: vertexSet_) {
        for (Edge *edge: orig->getAdj()) {
            Vertex *dest = edge->getDest();
            matrix[orig->getId()][dest->getId()] = edge->getWeight();
        }
    }
    for (int i = 0; i < (int)vertexSet_.size(); i++) {
        for (int j = 0; j < (int)vertexSet_.size(); j++) {
            if (isnan(matrix[i][j]))
                matrix[i][j] = haversineDistance(findVertex(i), findVertex(j));
        }
    }
    return matrix;
}

template<class T>
void Graph::deleteMatrix(T **matrix) const {
    for (int i = 0; i < (int)vertexSet_.size(); i++)
        delete [] matrix[i];
    delete [] matrix;
}

bool Graph::respectsTriangularInequality() {
    double **dist = getDistMatrix();
    for (int w = 0; w < (int)vertexSet_.size(); w++) {
        for (int v = 0; v < (int)vertexSet_.size(); v++) {
            for (int u = 0; u < w; u++) {
                if (dist[u][w] > dist[u][v] + dist[v][w]) {
                    deleteMatrix(dist);
                    return false;
                }
            }
        }
    }

    deleteMatrix(dist);
    return true;
}

void Graph::kruskal(std::vector<Edge*> &edges) {
    if (vertexSet_.empty())
        return;

    Ufds ufds((int)vertexSet_.size());
    sort(edges.begin(), edges.end(), [](Edge *edge1, Edge *edge2) {
        return edge1->getWeight() < edge2->getWeight();
    });

    for (Edge *edge: edges) {
        Vertex *u = edge->getOrig(), *v = edge->getDest();
        if (!ufds.isSameSet(u->getId(), v->getId())) {
            edge->setSelected(true);
            edge->getReverse()->setSelected(true);
            ufds.linkSets(u->getId(), v->getId());
        }
    }

    for (Vertex *vertex: vertexSet_) {
        vertex->setVisited(false);
        vertex->setDegree(0);
        vertex->setPath(nullptr);
    }

    Vertex *root = vertexSet_[0];
    kruskalDfs(root);
}

void Graph::kruskalDfs(Vertex *v) {
    v->setVisited(true);
    for (Edge *e: v->getAdj()) {
        if (!e->isSelected())
            continue;

        Vertex *w = e->getDest();
        if (!w->isVisited()) {
            w->setPath(e->getReverse());
            v->setDegree(v->getDegree() + 1);
            w->setDegree(w->getDegree() + 1);
            kruskalDfs(w);
        }
    }
}

void Graph::minWeightPerfectMatchingGreedy(const vector<Edge *> &sortedEdges) {
    for (Edge *edge: sortedEdges) {
        Vertex *u = edge->getOrig(), *v = edge->getDest();
        if (u->isVisited() || v->isVisited())
            continue;
        edge->setSelected(true);
        u->setVisited(true);
        v->setVisited(true);
    }
}

double Graph::hamiltonianCircuitDfs(Vertex *vertex, Vertex *&last) {
    double length = 0;
    vertex->setVisited(true);

    for (Edge *edge: vertex->getAdj()) {
        Vertex *dest = edge->getDest();
        if (edge->isSelected() && !edge->getDest()->isVisited()) {
            length += last->findEdgeTo(dest->getId())->getWeight();
            last = dest;
            length += hamiltonianCircuitDfs(dest, last);
        }
    }

    return length;
}

Graph *Graph::createCompleteCopy() const {
    auto newGraph = new Graph;
    for (Vertex *vertex: vertexSet_)
        newGraph->addVertex(new Vertex(vertex->getId(), vertex->getLatitude(), vertex->getLongitude()));

    auto hasEdgeTo = new bool[vertexSet_.size()];
    for (Vertex *vertex: vertexSet_) {
        for (int id = vertex->getId() + 1; id < (int)vertexSet_.size(); id++)
            hasEdgeTo[id] = false;
        for (Edge *edge: vertex->getAdj()) {
            if (edge->getDest()->getId() < vertex->getId())
                continue;
            newGraph->addEdge(vertex->getId(), edge->getDest()->getId(), edge->getWeight());
            hasEdgeTo[edge->getDest()->getId()] = true;
        }
        for (int id = vertex->getId() + 1; id < (int)vertexSet_.size(); id++) {
            if (!hasEdgeTo[id])
                newGraph->addEdge(vertex->getId(), id, haversineDistance(vertex, findVertex(id)));
        }
    }

    delete [] hasEdgeTo;
    return newGraph;
}
