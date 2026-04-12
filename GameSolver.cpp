#include "GameSolver.h"
#include "SetUtils.h"
#include <iostream>
#include <algorithm>
#include <climits>
#include <queue>

// Player-0 predecessors where every successor remains inside states.
std::unordered_set<int> prePlayer0AllSuccessorsIn(const GameGraph& game, const std::unordered_set<int>& states) {
    std::unordered_set<int> result;
    for (auto v : game.player0) {
        bool allIn = true;
        for (int w : game.adjacency[v]) {
            if (states.find(w) == states.end()) {
                allIn = false;
                break;
            }
        }
        if (allIn)
            result.insert(v);
    }
    return result;
}

// Player-1 predecessors with at least one successor inside states.
std::unordered_set<int> prePlayer1HasSuccessorIn(const GameGraph& game, const std::unordered_set<int>& states) {
    std::unordered_set<int> result;
    for (auto v : game.player1)  {
        for (int w : game.adjacency[v]) {
            if (states.find(w) != states.end()) {
                result.insert(v);
                break;
            }
        }
    }
    return result;
}

std::unordered_set<int> controllablePredecessor(const GameGraph& game, const std::unordered_set<int>& states) {
    return setUnion(prePlayer0AllSuccessorsIn(game, states), prePlayer1HasSuccessorIn(game, states));
}

std::unordered_set<int> livePrePlayer1AllIn(const GameGraph& game, const std::unordered_set<int>& states) {
    std::unordered_set<int> result;
    for (auto v : game.player1) {
        if (!game.liveEdges[v].empty()) {
            bool allLiveIn = true;
            for (int w : game.liveEdges[v]) {
                if (states.find(w) == states.end()) {
                    allLiveIn = false;
                    break;
                }
            }
            if (allLiveIn)
                result.insert(v);
        }
    }
    return result;
}

std::unordered_set<int> getLivePlayer1Vertices(const GameGraph& game) {
    std::unordered_set<int> liveVertices;
    for (auto v : game.player1) {
        if (!game.liveEdges[v].empty()) {
            liveVertices.insert(v);
        }
    }
    return liveVertices;
}

std::pair<std::unordered_set<int>, std::vector<int> > computeWinningSet(GameGraph &game, const std::unordered_set<int>& buchi) {
    // Progress measure: first outer-iteration index at which each state appears in Y.
    std::vector<int> prog = std::vector<int>(game.n, -1);
    std::unordered_set<int> universal;
    for (auto i: game.player0) universal.insert(i);
    for (auto i: game.player1) universal.insert(i);
    std::unordered_set<int> liveVertices = getLivePlayer1Vertices(game);
    
    std::unordered_set<int> Y;
    auto complementG = setDifference(universal, buchi);
    auto complementLiveVertices = setDifference(universal, liveVertices);

    int iteration = 0;
    while (true) {
        std::cout << "iteration " << iteration << std::endl;
        std::unordered_set<int> X = universal;
        while (true) {
            // Inner greatest fixed-point for the current outer approximation Y.
            auto cpreY = controllablePredecessor(game, Y);
            auto left = setUnion(complementG, cpreY);
            auto cpreX = controllablePredecessor(game, X);
            auto livePreX = livePrePlayer1AllIn(game, X);
            auto pre1Y = prePlayer1HasSuccessorIn(game, Y);
            auto right = setUnion(livePreX, complementLiveVertices);
            right = setUnion(right, pre1Y);
            auto inner = setIntersection(cpreX, right);
            auto X_next = setIntersection(left, inner);
            
            if (X_next == X) break;
            else X = X_next;
        }
        if (X == Y) break;
        else Y = X;
        
        for (auto v : Y) {
            if (prog[v] == -1) prog[v] = iteration;
        }
        iteration++;
    }
    for (int v = 0; v < game.n; ++v)
        if (game.findPlayer(v) == -1)
            prog[v] = 0;

    return std::make_pair(Y, prog);
}

int liftProgressMeasure(int v, const GameGraph& game, const std::vector<int>& p, const std::unordered_set<int>& buchi) {
    int baseVal = 0;

    if (game.findPlayer(v) == 0) {
        int mx = INT_MIN;
        for (int w : game.adjacency[v])
            mx = std::max(mx, p[w]);
        baseVal = mx;
    } else {
        if ((!game.liveEdges[v].empty()) && (buchi.find(v) == buchi.end())) {
            int maxLive = INT_MIN;
            for (int u : game.liveEdges[v]) maxLive = std::max(maxLive, p[u]);
            int minALL = INT_MAX;
            for (int w : game.adjacency[v]) minALL = std::min((p[w] + 1), minALL);
            baseVal = std::min(minALL, maxLive);
        } else {
            int mn = INT_MAX;
            for (int w : game.adjacency[v]) mn = std::min(mn, p[w]);
            baseVal = mn;
        }
    }

    int result = baseVal;
    if (buchi.find(v) != buchi.end()) result += 1;
    return result;
}

std::vector<std::vector<int> > buildReverseAdjacency(const GameGraph& game) {
    std::vector<std::vector<int> > pred(game.n);
    for (int v = 0; v < game.n; ++v) {
        if (game.findPlayer(v) == -1) continue;
        for (auto w : game.adjacency[v])
            pred[w].push_back(v);
    }
    return pred;
}

void predecessorsLifting(const GameGraph& game, std::vector<int>& prog, const std::unordered_set<int>& buchi, int T) {
    auto pred = buildReverseAdjacency(game);
    std::vector<bool> inQueue(game.n, false);
    std::queue<int> Q;

    for (auto v : game.startLift) {
        if (prog[v] < T) {
            Q.push(v);
            inQueue[v] = true;
        }
    }
    std::unordered_set<int> updated;
    int num = 0;
    // Update propagation via reverse edges.
    while (!Q.empty()) {
        int v = Q.front();
        Q.pop();
        inQueue[v] = false; 
        int oldVal = prog[v];
        if (oldVal >= T) continue;
        int newVal = liftProgressMeasure(v, game, prog, buchi);
        newVal = std::min(newVal, T);
        
        if (newVal > oldVal) {
            updated.insert(v);
            num++;
            prog[v] = newVal;
            for (int u : pred[v]) {
                if (prog[u] < T && !inQueue[u]) {
                    Q.push(u);
                    inQueue[u] = true;
                }
            }
        }
    }
    std::cout << "Total number of vertices lifted: " << updated.size() << std::endl;
    std::cout << "number of lifts: " << num << std::endl;
}
