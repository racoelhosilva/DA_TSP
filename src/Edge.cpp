#include "Edge.h"

Edge::Edge(Vertex *orig, Vertex *dest, double weight) : orig(orig), dest(dest), weight(weight), reverse(nullptr) {}

Edge::~Edge() = default;

Vertex *Edge::getDest() const {
    return dest;
}

Vertex *Edge::getOrig() const {
    return orig;
}

double Edge::getWeight() const {
    return weight;
}

bool Edge::isSelected() const {
    return selected;
}

Edge *Edge::getReverse() const {
    return reverse;
}

void Edge::setSelected(bool selected) {
    Edge::selected = selected;
    if (reverse != nullptr)
        reverse->selected = selected;
}

void Edge::setReverse(Edge *reverse) {
    Edge::reverse = reverse;
}
