#pragma once

#include <vector>
#include <unordered_set>

// Returns all valid neighbors in a square radius around a 1-based grid state.
// Includes a single sentinel 0 if any neighbor falls outside the grid.
// State 0 is a trap state.
std::vector<int> collectReachableNeighbors(int stateIndex, int radius, int gridSize, std::unordered_set<int>& unsafeStates);

// Random low-data-state refinement experiment.
void EXP_RAND(int size, int badStates, int normal_oa, int low_data_oa);
// Single low-data-state refinement experiment.
void EXP_ADD1(int size, int normal_oa, int low_data_oa);
// Multi-room staged refinement experiment.
void EXP_ROOM(int size, int normal_oa, int low_data_oa);
// Data-driven system learning experiment.
void EXP_LEARNING(std::vector<double> lb, std::vector<double> ub, int n);
