#include "GameGraph.h"

// Pre-allocate all graph arrays.
GameGraph::GameGraph(int numStates)
    : n(numStates), vmin(numStates), vmax(numStates), adjacency(numStates), liveEdges(numStates) {}

// Add a non-live transition.
void GameGraph::addEdge(int v, int w) {
    adjacency[v].push_back(w);
}

// Add a live transition.
void GameGraph::addLiveEdge(int v, int w) {
    adjacency[v].push_back(w);
    liveEdges[v].push_back(w);
}

// Resolve owner set membership for a vertex.
int GameGraph::findPlayer(int v) const {
    if (player0.find(v) != player0.end()) {
        return 0;
    } else if (player1.find(v) != player1.end()) {
        return 1;
    } else {
        return -1;
    }
}

bool GameGraph::inSet(const std::unordered_set<int>& S, int v) const {
    return (S.find(v) != S.end());
}
