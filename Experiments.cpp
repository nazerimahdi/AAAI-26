#include "Experiments.h"
#include "GameGraph.h"
#include "GameSolver.h"
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>

extern std::mt19937 gen;

// Collect neighborhoods around a grid state while respecting the grid boundaries and unsafe states. A sentinel value of 0 is used to indicate out-of-bounds or unsafe neighbors, but only one sentinel is added per neighborhood.
std::vector<int> collectReachableNeighbors(int stateIndex, int radius, int gridSize, std::unordered_set<int>& unsafeStates) {
    std::vector<int> neighbors;
    bool hasPushedBoundarySentinel = false;

    stateIndex -= 1; // Convert from 1-based state ids to 0-based grid coordinates.
    int row = stateIndex / gridSize;
    int col = stateIndex % gridSize;

    for (int i = row - radius; i <= row + radius; ++i) {
        for (int j = col - radius; j <= col + radius; ++j) {
            int neighbor = i * gridSize + j + 1;
            if (neighbor >= 1 && neighbor <= gridSize * gridSize && i >= 0 && j >= 0 && i < gridSize && j < gridSize && unsafeStates.find(neighbor) == unsafeStates.end()) {
                neighbors.push_back(neighbor);
            } else if (!hasPushedBoundarySentinel) {
                // One boundary sentinel per neighborhood.
                hasPushedBoundarySentinel = true;
                neighbors.push_back(0);
            }
        }
    }
    return neighbors;
}

void EXP_RAND(int size, int badStates, int normal_oa, int low_data_oa) {
    // Metrics vector: [grid size, number of low-data states, graph size, solve/lift timings...].
    std::vector<double> resu;
    resu.push_back(size);
    resu.push_back(badStates);

    // Goal region near the top-left corner.
    std::unordered_set<int> G;
    G.insert(size * 1 + 2);
    G.insert(size * 1 + 3);
    G.insert(size * 1 + 4);
    
    G.insert(size * 2 + 2);
    G.insert(size * 2 + 3);
    G.insert(size * 2 + 4);

    G.insert(size * 3 + 2);
    G.insert(size * 3 + 3);
    G.insert(size * 3 + 4);

    // Unsafe cross-shaped obstacle around the center.
    std::unordered_set<int> unsafe;
    for (int i = size/2 - 3; i < size/2 + 3; ++i) {
        for (int j = size/2 - 1; j < size/2 + 1; ++j) {
            int state = i * size + j + 1;
                unsafe.insert(state);
        }
    }
    for (int i = size/2 - 3; i < size/2 + 3; ++i) {
        for (int j = size/2 - 1; j < size/2 + 1; ++j) {
            int state = j * size + i + 1;
                unsafe.insert(state);
        }
    }
    std::uniform_int_distribution<int> dist(1, size * size);
    // Low-data states with conservative over- and under-approximations.
    std::unordered_set<int> low_data;
    // low_data.insert(size * size);
    for (int i = 0; i < badStates; ++i) {
        int state = dist(gen);
        while (G.find(state) != G.end() || unsafe.find(state) != unsafe.end() || low_data.find(state) != low_data.end())
            state = dist(gen);
        low_data.insert(state);
    }

    // Visualize the environment.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else if (low_data.find(state) != low_data.end())
                std::cout << "L" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << "\n";
    }

    // Build the game graph.
    GameGraph game(2000000);

    game.player0.insert(0);
    game.addEdge(0, 0);
    int v = size * size + 1; 

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            game.player0.insert(state);

            if (unsafe.find(state) != unsafe.end()) {
                game.addEdge(state, 0);
                continue;
            }

            game.vmin[state] = v;
            std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
            for (auto nextState : nextStates) {
                if (nextState != 0) {
                    int actionNode = v++;
                    game.player1.insert(actionNode);
                    game.addEdge(state, actionNode);

                    int oa = normal_oa;
                    if (low_data.find(state) != low_data.end())
                        oa = low_data_oa;
                    std::vector<int> OA = collectReachableNeighbors(nextState, oa, size, unsafe);

                    int envNode = v++;
                    // to under approximation
                    if (low_data.find(state) == low_data.end()) {
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        game.addLiveEdge(envNode, nextState);
                    }

                    for (auto neighbor : OA) {
                        if (neighbor == state)
                            continue;
                        int envNode = v++;
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        if (low_data.find(state) == low_data.end())
                            game.addLiveEdge(envNode, nextState);
                        game.addLiveEdge(envNode, neighbor);
                    }
                }
            }
            game.vmax[state] = v - 1;
        }
    }
    resu.push_back(game.player0.size() + game.player1.size());
    std::cout << "Game graph built with " << game.player0.size() + game.player1.size() << " states.\n";
    game.n = v;
    
    // Initial fixed-point solve.
    auto tic = std::chrono::steady_clock::now();
    auto [W, prog] = computeWinningSet(game, G);
    auto tok = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = tok - tic;
    std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;
    resu.push_back(diff.count());
    std::cout << "Winning region under fair-adversarial Buchi: ";
    for (auto v : W)
        if (v <= size * size && unsafe.find(v) == unsafe.end())
            std::cout << v << " ";
    std::cout << std::endl;

    // Visualize the winning region.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (W.find(state) != W.end())
                std::cout << "F" << " ";
            else if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << std::endl;
    }

    // Upper cap used by the progress-measure lifting phase.
    int T = 1 + size * size + G.size();
    std::cout << "T=" << T << "\n"; 

    // This is to handle states with a self loop and no progress value assigned by computeWinningSet, which would otherwise be treated as -1 and cause issues during lifting.
    for (int i = 0; i < game.n; ++i)
        if (prog[i] == -1)
            prog[i] = T;

    // Refine each low-data state by rebuilding only its local action/environment gadget.
    for (auto state : low_data) {
        if (unsafe.find(state) != unsafe.end())
            continue;

        game.adjacency[state].clear();
        game.liveEdges[state].clear();
        for (int i = game.vmin[state]; i <= game.vmax[state]; ++i) {
            game.adjacency[i].clear();
            game.liveEdges[i].clear();
            if (game.findPlayer(i) == 0)
                game.player0.erase(game.player0.find(i));
            else if (game.findPlayer(i) == 1)
                game.player1.erase(game.player1.find(i));
        }

        int v = game.vmin[state];
        std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
        for (auto nextState : nextStates) {
            if (nextState != 0) {
                int actionNode = v++;
                game.player1.insert(actionNode);
                game.addEdge(state, actionNode);
                game.startLift.push_back(actionNode);

                std::vector<int> OA = collectReachableNeighbors(nextState, low_data_oa, size, unsafe);
                int envNode = v++;
                game.player1.insert(envNode);
                game.addEdge(actionNode, envNode);
                game.addLiveEdge(envNode, nextState);
                game.startLift.push_back(envNode);
                std::vector<int> newOA = collectReachableNeighbors(nextState, normal_oa, size, unsafe);
                for (auto neighbor : OA) {
                    if (neighbor == state)
                        continue;
                    int envNode = v++;
                    if (std::find(newOA.begin(), newOA.end(), neighbor) != newOA.end()) {
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        game.addLiveEdge(envNode, nextState);
                        game.addLiveEdge(envNode, neighbor);
                        game.startLift.push_back(envNode);
                    }
                }
            }
        }
    }

    // Incremental lifting after local refinements.
    tic = std::chrono::steady_clock::now();
    predecessorsLifting(game, prog, G, T);
    tok = std::chrono::steady_clock::now();
    diff = tok - tic;
    std::cout << "Lifting execution time: " << diff.count() << " s" << std::endl;
    resu.push_back(diff.count());
    
    std::cout << "After lifting: " << std::endl;
    for (int i = 0; i <= size * size; ++i) {
        if (prog[i] < T && unsafe.find(i) == unsafe.end())
            std::cout << i << " ";
    }
    std::cout << std::endl;
    
    // Recompute winning domain using the fixed-point on the refined graph for comparison.
    tic = std::chrono::steady_clock::now();
    auto [W1, prog1] = computeWinningSet(game, G);
    tok = std::chrono::steady_clock::now();
    diff = tok - tic;
    std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;
    std::cout << "Winning region under fair-adversarial Buchi: ";
    for (auto v : W1)
        if (v <= size * size && unsafe.find(v) == unsafe.end())
            std::cout << v << " ";
    std::cout << std::endl;
    resu.push_back(diff.count());

    for (auto res: resu)
        std::cout << res << " ";
    std::cout << std::endl;
}

void EXP_ADD1(int size, int normal_oa, int low_data_oa) {
    // Metrics vector: [grid size, graph size, solve/lift timings...].
    std::vector<double> resu;
    resu.push_back(size);

    // Compact goal region.
    std::unordered_set<int> G;
    G.insert(size * 1 + 3);
    G.insert(size * 1 + 2);
    G.insert(size * 2 + 3);
    G.insert(size * 2 + 2);

    // Unsafe cross-shaped obstacle around the center.
    std::unordered_set<int> unsafe;
    for (int i = size/2 - 3; i < size/2 + 3; ++i) {
        for (int j = size/2 - 1; j < size/2 + 1; ++j) {
            int state = i * size + j + 1;
                unsafe.insert(state);
        }
    }
    for (int i = size/2 - 3; i < size/2 + 3; ++i) {
        for (int j = size/2 - 1; j < size/2 + 1; ++j) {
            int state = j * size + i + 1;
                unsafe.insert(state);
        }
    }
    // Single low-data state.
    std::unordered_set<int> low_data;
    low_data.insert(size * size);

    // Visualize the environment.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else if (low_data.find(state) != low_data.end())
                std::cout << "L" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << std::endl;
    }

    // Build the game graph.
    GameGraph game(2000000);

    game.player0.insert(0);
    game.addEdge(0, 0);
    int v = size * size + 1; 

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            game.player0.insert(state);

            if (unsafe.find(state) != unsafe.end()) {
                game.addEdge(state, 0);
                continue;
            }

            game.vmin[state] = v;
            std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
            for (auto nextState : nextStates) {
                if (nextState != 0) {
                    int actionNode = v++;
                    game.player1.insert(actionNode);
                    game.addEdge(state, actionNode);

                    int oa = normal_oa;
                    if (low_data.find(state) != low_data.end())
                        oa = low_data_oa;
                    std::vector<int> OA = collectReachableNeighbors(nextState, oa, size, unsafe);

                    int envNode = v++;
                    // to under approximation
                    if (low_data.find(state) == low_data.end()) {
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        game.addLiveEdge(envNode, nextState);
                    }

                    for (auto neighbor : OA) {
                        if (neighbor == state)
                            continue;
                        int envNode = v++;
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        if (low_data.find(state) == low_data.end())
                            game.addLiveEdge(envNode, nextState);
                        game.addLiveEdge(envNode, neighbor);
                    }
                }
            }
            game.vmax[state] = v - 1;
        }
    }
    resu.push_back(game.player0.size() + game.player1.size());
    std::cout << "Game graph built with " << game.player0.size() + game.player1.size() << " states.\n";
    game.n = v;
    
    // Initial fixed-point solve.
    auto tic = std::chrono::steady_clock::now();
    auto [W, prog] = computeWinningSet(game, G);
    auto tok = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = tok - tic;
    std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;
    resu.push_back(diff.count());
    std::cout << "Winning region under fair-adversarial Buchi: ";
    for (auto v : W)
        if (v <= size * size && unsafe.find(v) == unsafe.end())
            std::cout << v << " ";
    std::cout << std::endl;

    // Visualize the winning region.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (W.find(state) != W.end())
                std::cout << "F" << " ";
            else if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << std::endl;
    }

    // Upper cap used by the progress-measure lifting phase.
    int T = 1 + size * size + G.size();
    std::cout << "T=" << T << "\n"; 

    // This is to handle states with a self loop and no progress value assigned by computeWinningSet, which would otherwise be treated as -1 and cause issues during lifting.
    for (int v = 0; v < game.n; ++v)
        if (prog[v] == -1)
                prog[v] = T;

    // update the game graph after new data arrives for the single low-data state, then incrementally lift.
    for (auto state : low_data) {
        if (unsafe.find(state) != unsafe.end())
            continue;

        game.adjacency[state].clear();
        game.liveEdges[state].clear();
        for (int i = game.vmin[state]; i <= game.vmax[state]; ++i) {
            game.adjacency[i].clear();
            game.liveEdges[i].clear();
            if (game.findPlayer(i) == 0)
                game.player0.erase(game.player0.find(i));
            else if (game.findPlayer(i) == 1)
                game.player1.erase(game.player1.find(i));
        }

        int v = game.vmin[state];
        std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
        for (auto nextState : nextStates) {
            if (nextState != 0) {
                int actionNode = v++;
                game.player1.insert(actionNode);
                game.addEdge(state, actionNode);
                game.startLift.push_back(actionNode);
                std::vector<int> OA = collectReachableNeighbors(nextState, low_data_oa, size, unsafe);
                int envNode = v++;
                game.player1.insert(envNode);
                game.addEdge(actionNode, envNode);
                game.addLiveEdge(envNode, nextState);
                game.startLift.push_back(envNode);

                std::vector<int> newOA = collectReachableNeighbors(nextState, normal_oa, size, unsafe);
                for (auto neighbor : OA) {
                    if (neighbor == state)
                        continue;
                    int envNode = v++;
                    if (std::find(newOA.begin(), newOA.end(), neighbor) != newOA.end()) {
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        game.addLiveEdge(envNode, nextState);
                        game.addLiveEdge(envNode, neighbor);
                        game.startLift.push_back(envNode);
                    }
                }
            }
        }
    }

    // Incremental lifting after local refinement.
    tic = std::chrono::steady_clock::now();
    predecessorsLifting(game, prog, G, T);
    tok = std::chrono::steady_clock::now();
    diff = tok - tic;
    std::cout << "Lifting execution time: " << diff.count() << " s" << std::endl;
    resu.push_back(diff.count());
    
    std::cout << "After lifting: " << std::endl;
    for (int i = 0; i <= size * size; ++i)
        if (prog[i] < T && unsafe.find(i) == unsafe.end())
            std::cout << i << " ";
    std::cout << std::endl;

    // Recompute winning set from scratch on the refined graph.
    tic = std::chrono::steady_clock::now();
    auto [W1, prog1] = computeWinningSet(game, G);
    tok = std::chrono::steady_clock::now();
    diff = tok - tic;
    std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;
    resu.push_back(diff.count());

    std::cout << "Winning region under fair-adversarial Buchi: ";
    for (auto v : W1)
        if (v <= size * size && unsafe.find(v) == unsafe.end())
            std::cout << v << " ";
    std::cout << std::endl;

    for (auto res: resu)
        std::cout << res << " ";
    std::cout << std::endl;
}

void EXP_ROOM(int size, int normal_oa, int low_data_oa) {
    // Goal region in the upper-left room.
    std::unordered_set<int> G;
    G.insert(size * 3 + 3);
    G.insert(size * 3 + 4);
    G.insert(size * 4 + 3);
    G.insert(size * 4 + 4);

    // Walls/corridors that shape a multi-room map.
    std::unordered_set<int> unsafe;

    int d = size / 3;
    for (int i = size - d; i < size; i++) {
        for (int j = d; j <= d+1; ++j) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }
    for (int i = size - d + 7; i < size; i++) {
        for (int j = 2*d; j <= 2*d+1; ++j) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }
    for (int j = size - d; j < size; j++) {
        for (int i = d; i <= d+1; ++i) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }

    for (int i = size - d; i <= size - d + 1; ++i) {
        for (int j = 5; j <= d; ++j) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }

    for (int i = size - d; i <= size - d + 1; ++i) {
        for (int j = d + 7; j < size; ++j) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }

    for (int i = 5; i <= 2 * d - 6; ++i) {
        for (int j = size - d; j <= size - d + 1; ++j) {
            int state = i * size + j + 1;
            unsafe.insert(state);
        }
    }

    // Define 5 low-data rooms.
    std::unordered_set<int> room[5];
    for (int i = 0; i < d ; ++i) {
        for (int j = size - d; j < size; ++j) {
            int state = i * size + j + 1;
            room[0].insert(state);
        }
    }
    for (int i = d; i < 2*d ; ++i) {
        for (int j = size - d; j < size; ++j) {
            int state = i * size + j + 1;
            room[1].insert(state);
        }
    }
    for (int i = 2*d; i < size; ++i) {
        for (int j = size - d + 1; j < size; ++j) {
            int state = i * size + j + 1;
            room[2].insert(state);
        }
    }
    for (int i = size - d; i <= size - 1; ++i) {
        for (int j = 0; j < d; ++j) {
            int state = i * size + j + 1;
            room[3].insert(state);
        }
    }
    for (int i = size - d; i <= size - 1; ++i) {
        for (int j = d; j <= d+d; ++j) {
            int state = i * size + j + 1;
            room[4].insert(state);
        }
    }

    // Union of all low-data rooms.
    std::unordered_set<int> low_data;
    for (int i = 0; i < 5; i++)
        for (auto state : room[i])
            low_data.insert(state);

    // Visualize the environment.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else if (room[0].find(state) != room[0].end())
                std::cout << "0" << " ";
            else if (room[1].find(state) != room[1].end())
                std::cout << "1" << " ";
            else if (room[2].find(state) != room[2].end())
                std::cout << "2" << " ";
            else if (room[3].find(state) != room[3].end())
                std::cout << "3" << " ";
            else if (room[4].find(state) != room[4].end())
                std::cout << "4" << " ";
            else if (low_data.find(state) != low_data.end())
                std::cout << "L" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << std::endl;
    }

    // Build the game graph.
    GameGraph game(2000000);

    game.player0.insert(0);
    game.addEdge(0, 0);
    int v = size * size + 1; 

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            game.player0.insert(state);

            if (unsafe.find(state) != unsafe.end()) {
                game.addEdge(state, 0);
                continue;
            }

            game.vmin[state] = v;
            std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
            for (auto nextState : nextStates) {
                if (nextState != 0) {
                    int actionNode = v++;
                    game.player1.insert(actionNode);
                    game.addEdge(state, actionNode);

                    int oa = normal_oa;
                    if (low_data.find(state) != low_data.end())
                        oa = low_data_oa;
                    std::vector<int> OA = collectReachableNeighbors(nextState, oa, size, unsafe);

                    // to under approximation
                    int envNode = v++;
                    if (low_data.find(state) == low_data.end()) {
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        game.addLiveEdge(envNode, nextState);
                    }

                    for (auto neighbor : OA) {
                        if (neighbor == state)
                            continue;
                        int envNode = v++;
                        game.player1.insert(envNode);
                        game.addEdge(actionNode, envNode);
                        if (low_data.find(state) == low_data.end())
                            game.addLiveEdge(envNode, nextState);
                        game.addLiveEdge(envNode, neighbor);
                    }
                }
            }
            game.vmax[state] = v - 1;
        }
    }
    std::cout << "Game graph built with " << game.player0.size() + game.player1.size() << " states.\n";
    game.n = v;
    
    // Initial fixed-point.
    auto tic = std::chrono::steady_clock::now();
    auto [W, prog] = computeWinningSet(game, G);
    auto tok = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = tok - tic;
    std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;

    std::cout << "Winning region under fair-adversarial Buchi: ";
    for (auto v : W)
        if (v <= size * size && unsafe.find(v) == unsafe.end())
            std::cout << v << " ";
    std::cout << std::endl;

    // Visualize the winning region.
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int state = i * size + j + 1;
            if (W.find(state) != W.end())
                std::cout << "F" << " ";
            else if (G.find(state) != G.end())
                std::cout << "G" << " ";
            else if (unsafe.find(state) != unsafe.end())
                std::cout << "X" << " ";
            else if (low_data.find(state) != low_data.end())
                std::cout << "L" << " ";
            else 
                std::cout << "." << " ";
        }
        std::cout << std::endl;
    }

    // Upper cap used by the progress-measure lifting phase.
    int T = 1 + size * size + G.size();
    std::cout << "T=" << T << "\n"; 

    // This is to handle states with a self loop and no progress value assigned by computeWinningSet, which would otherwise be treated as -1 and cause issues during lifting.
    for (int i = 0; i < game.n; ++i)
        if (prog[i] == -1)
            prog[i] = T;

    // Obtain data for each room one by one and lift after each stage.
    for (int roomn = 4; roomn >= 0; roomn--) {
        for (auto state : room[roomn]) {
            if (unsafe.find(state) != unsafe.end())
                continue;

            game.adjacency[state].clear();
            game.liveEdges[state].clear();
            for (int i = game.vmin[state]; i <= game.vmax[state]; ++i) {
                game.adjacency[i].clear();
                game.liveEdges[i].clear();
                if (game.findPlayer(i) == 0)
                    game.player0.erase(game.player0.find(i));
                else if (game.findPlayer(i) == 1)
                    game.player1.erase(game.player1.find(i));
            }

            int v = game.vmin[state];
            std::vector<int> nextStates = collectReachableNeighbors(state, 2, size, unsafe); // actions
            for (auto nextState : nextStates) {
                if (nextState != 0) {
                    int actionNode = v++;
                    game.player1.insert(actionNode);
                    game.addEdge(state, actionNode);
                    game.startLift.push_back(actionNode);

                    std::vector<int> OA = collectReachableNeighbors(nextState, low_data_oa, size, unsafe);
                    int envNode = v++;
                    game.player1.insert(envNode);
                    game.addEdge(actionNode, envNode);
                    game.addLiveEdge(envNode, nextState);
                    game.startLift.push_back(envNode);

                    std::vector<int> newOA = collectReachableNeighbors(nextState, normal_oa, size, unsafe);
                    for (auto neighbor : OA) {
                        if (neighbor == state)
                            continue;
                        int envNode = v++;
                        if (std::find(newOA.begin(), newOA.end(), neighbor) != newOA.end()) {
                            game.player1.insert(envNode);
                            game.addEdge(actionNode, envNode);
                            game.addLiveEdge(envNode, nextState);
                            game.addLiveEdge(envNode, neighbor);
                            game.startLift.push_back(envNode);
                        }
                    }
                }
            }
        }

        // Lift progress values after local refinements.
        tic = std::chrono::steady_clock::now();
        predecessorsLifting(game, prog, G, T);
        tok = std::chrono::steady_clock::now();
        diff = tok - tic;
        std::cout << "Lifting execution time: " << diff.count() << " s" << std::endl;


        std::cout << "After lifting: " << std::endl;
        for (int i = 0; i <= size * size; ++i) {
            if (prog[i] < T && unsafe.find(i) == unsafe.end())
                std::cout << i << " ";
        }
        std::cout << std::endl;

        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                int state = i * size + j + 1;
                if (prog[state] < T)
                    std::cout << "F" << " ";
                else if (G.find(state) != G.end())
                    std::cout << "G" << " ";
                else if (unsafe.find(state) != unsafe.end())
                    std::cout << "X" << " ";
                else 
                    std::cout << "." << " ";
            }
            std::cout << std::endl;
        }

        // Fixed-point recomputation after each staged refinement for comparison.
        tic = std::chrono::steady_clock::now();
        auto [W1, prog1] = computeWinningSet(game, G);
        tok = std::chrono::steady_clock::now();
        diff = tok - tic;
        std::cout << "computeWinningSet execution time: " << diff.count() << " s" << std::endl;

        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                int state = i * size + j + 1;
                if (W1.find(state) != W1.end())
                    std::cout << "F" << " ";
                else if (G.find(state) != G.end())
                    std::cout << "G" << " ";
                else if (unsafe.find(state) != unsafe.end())
                    std::cout << "X" << " ";
                else if (low_data.find(state) != low_data.end())
                    std::cout << "L" << " ";
                else 
                    std::cout << "." << " ";
            }
            std::cout << std::endl;
        }
    }
}

std::pair<double, double> nsup = {-1.5, 1.5};

// Noisy system dynamics only used for data generation.
std::vector<double> system_dynamic(std::vector<double> &x, std::vector<double> &u) {
    std::uniform_real_distribution<double> dist(nsup.first, nsup.second);
    return std::vector<double> {x[0] + 10 * u[0] * cos(u[1]) + dist(gen), x[1] + 10 * u[0] * sin(u[1]) + dist(gen)};
}

// One dynamics sample (state, action, next state).
struct sample {
    std::vector<double> x;
    std::vector<double> u;
    std::vector<double> x_next;
};

const std::vector<std::vector<double> > U = {
    {-0.2, 0.0}, {-0.2, M_PI/8}, {-0.2, M_PI/4}, {-0.2, 3*M_PI/8}, {-0.2, M_PI/2},
    {-0.2, 5*M_PI/8}, {-0.2, 3*M_PI/4}, {-0.2, 7*M_PI/8}, {-0.2, M_PI},

    {-0.1, 0.0}, {-0.1, M_PI/8}, {-0.1, M_PI/4}, {-0.1, 3*M_PI/8}, {-0.1, M_PI/2},
    {-0.1, 5*M_PI/8}, {-0.1, 3*M_PI/4}, {-0.1, 7*M_PI/8}, {-0.1, M_PI},

    {0.1, 0.0}, {0.1, M_PI/8}, {0.1, M_PI/4}, {0.1, 3*M_PI/8}, {0.1, M_PI/2},
    {0.1, 5*M_PI/8}, {0.1, 3*M_PI/4}, {0.1, 7*M_PI/8}, {0.1, M_PI},

    {0.2, 0.0}, {0.2, M_PI/8}, {0.2, M_PI/4}, {0.2, 3*M_PI/8}, {0.2, M_PI/2},
    {0.2, 5*M_PI/8}, {0.2, 3*M_PI/4}, {0.2, 7*M_PI/8}, {0.2, M_PI}
};

std::vector<sample> dataset[36];
std::vector<double> SLB = {0.0, 0.0};
std::vector<double> SUB = {10.0, 10.0};


// Generate transition samples for every discrete control input.
void generate_dataset(std::vector<double> lb, std::vector<double> ub, int num) {
    std::uniform_real_distribution<double> dist_x0(lb[0], ub[0]);
    std::uniform_real_distribution<double> dist_x1(lb[1], ub[1]);
    for (int i = 0; i < 36; ++i) {
        for (int j = 0; j < num; ++j) {
            sample s;
            s.x = {dist_x0(gen), dist_x1(gen)};
            s.u = U[i];
            s.x_next = system_dynamic(s.x, s.u);
            dataset[i].push_back(s);
        }
    }
}

// Compute the over- and under-approximations for a state-control pair.
void learn_f(std::vector<double> lb, std::vector<double> ub, int uindex, int precision, double LX, int n) {
    std::vector<double> mincheckf = {1e9, 1e9};
    std::vector<double> maxhatf = {-1e9, -1e9};
    for (int i = 0; i < precision; i++) {
        for (int j = 0; j < precision; j++) {
            std::vector<double> checkf = {-1e9, -1e9};
            std::vector<double> hatf = {1e9, 1e9};
            double x0 = lb[0] + (i+0.5) * (ub[0] - lb[0]) / precision;
            double x1 = lb[1] + (j+0.5) * (ub[1] - lb[1]) / precision;
            std::vector<double> x = {x0, x1};
            for (int k = 0; k < n; ++k) {
                double dx0 = abs(x[0] - dataset[uindex][k].x[0]) + (ub[0] - lb[0]) / precision / 2;
                double dx1 = abs(x[1] - dataset[uindex][k].x[1]) + (ub[1] - lb[1]) / precision / 2;
                double dist = std::max(dx0, dx1);
                checkf[0] = std::max(checkf[0], dataset[uindex][k].x_next[0] - dist * LX - nsup.second);
                checkf[1] = std::max(checkf[1], dataset[uindex][k].x_next[1] - dist * LX - nsup.second);
                hatf[0] = std::min(hatf[0], dataset[uindex][k].x_next[0] + dist * LX - nsup.first);
                hatf[1] = std::min(hatf[1], dataset[uindex][k].x_next[1] + dist * LX - nsup.first);
            }
            mincheckf[0] = std::min(mincheckf[0], checkf[0]);
            mincheckf[1] = std::min(mincheckf[1], checkf[1]);
            maxhatf[0] = std::max(maxhatf[0], hatf[0]);
            maxhatf[1] = std::max(maxhatf[1], hatf[1]);
        }
    }
    std::cout <<  ((maxhatf[0] + nsup.second) - (mincheckf[0] + nsup.first)) * ((maxhatf[1] + nsup.second) - (mincheckf[1] + nsup.first)) << ", ";
    std::cout <<  std::max(((mincheckf[0] + nsup.second) - (maxhatf[0] + nsup.first)), 0.0) * std::max((mincheckf[1] + nsup.second) - (maxhatf[1] + nsup.first), 0.0)  << std::endl;
}

void EXP_LEARNING(std::vector<double> lb, std::vector<double> ub, int n) {
    // Compute over- and under-approximation areas as sample count increases.
    generate_dataset(lb, ub, n);
    for (int i = 0; i < 1000; ++i) {
        learn_f({0.0, 0.0}, {1.0, 1.0}, 0, 20, 1, i);  
    }
}
