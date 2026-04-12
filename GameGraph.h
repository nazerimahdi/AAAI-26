#pragma once

#include <vector>
#include <unordered_set>

/**
 * @brief Game graph structure representing the state space.
 */
class GameGraph {
public:
    int n;
    std::unordered_set<int> player0;
    std::unordered_set<int> player1;
    std::vector<std::vector<int> > adjacency;      // All outgoing edges.
    std::vector<std::vector<int> > liveEdges;      // Live (fair) edges.
    std::vector<int> vmin;
    std::vector<int> vmax;
    std::vector<int> startLift;

    /**
     * @brief Constructor for the GameGraph.
     * @param numStates Number of states in the game graph.
     */
    GameGraph(int numStates);

    /**
     * @brief Add a regular edge to the game graph.
     * @param v Source vertex.
     * @param w Destination vertex.
     */
    void addEdge(int v, int w);

    /**
     * @brief Add a live (fair) edge to the game graph.
     * @param v Source vertex.
     * @param w Destination vertex.
     */
    void addLiveEdge(int v, int w);

    /**
     * @brief Returns the player owning the vertex.
     * @param v Vertex.
     * @return 0 for Player 0, 1 for Player 1, -1 if not found.
     */
    int findPlayer(int v) const;

    /**
     * @brief Check membership of a vertex in a set.
     * @param S The set.
     * @param v Vertex to find.
     * @return true if vertex is in the set, false otherwise.
     */
    bool inSet(const std::unordered_set<int>& S, int v) const;
};
