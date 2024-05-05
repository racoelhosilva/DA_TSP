#include "Edge.h"

Edge::Edge(Vertex *orig, Vertex *dest, double weight) : orig_(orig), dest_(dest), weight_(weight), reverse_(nullptr) {}

Edge::~Edge() = default;

Vertex *Edge::getDest() const {
    return dest_;
}

Vertex *Edge::getOrig() const {
    return orig_;
}

double Edge::getWeight() const {
    return weight_;
}

bool Edge::isSelected() const {
    return selected_;
}

Edge *Edge::getReverse() const {
    return reverse_;
}

void Edge::setSelected(bool selected) {
    selected_ = selected;
    if (reverse_ != nullptr)
        reverse_->selected_ = selected;
}

void Edge::setReverse(Edge *reverse) {
    reverse_ = reverse;
}
