#pragma once

#include "GameGraph.h"
#include <unordered_set>
#include <vector>
#include <utility>

// Predecessors in player0 where all successors stay inside states.
std::unordered_set<int> prePlayer0AllSuccessorsIn(const GameGraph& game, const std::unordered_set<int>& states);
// Predecessors in player1 where at least one successor is in states.
std::unordered_set<int> prePlayer1HasSuccessorIn(const GameGraph& game, const std::unordered_set<int>& states);
// One-step controllable predecessor operator.
std::unordered_set<int> controllablePredecessor(const GameGraph& game, const std::unordered_set<int>& states);
// Player1 vertices whose live successors all remain in states.
std::unordered_set<int> livePrePlayer1AllIn(const GameGraph& game, const std::unordered_set<int>& states);
// Player1 vertices that have at least one live edge.
std::unordered_set<int> getLivePlayer1Vertices(const GameGraph& game);

// Compute Buchi winning set and progress measure.
std::pair<std::unordered_set<int>, std::vector<int> > computeWinningSet(GameGraph &game, const std::unordered_set<int>& buchi);

// Lift one vertex under the fair-adversarial progress-measure update rule.
int liftProgressMeasure(
    int v,
    const GameGraph& game,
    const std::vector<int>& p,
    const std::unordered_set<int>& buchi
);

// Build reverse adjacency lists for efficient predecessor propagation.
std::vector<std::vector<int> > buildReverseAdjacency(const GameGraph& game);

// Incremental lifting.
void predecessorsLifting(
    const GameGraph& game,
    std::vector<int>& prog,
    const std::unordered_set<int>& buchi,
    int T
);
