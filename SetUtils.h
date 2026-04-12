#pragma once

#include <unordered_set>

/**
 * @brief Returns the union of two unordered sets.
 * 
 * @param A First set.
 * @param B Second set.
 * @return std::unordered_set<int> The union of A and B.
 */
inline std::unordered_set<int> setUnion(const std::unordered_set<int>& A,
                                        const std::unordered_set<int>& B) {
    std::unordered_set<int> result = A;
    result.insert(B.begin(), B.end());
    return result;
}

/**
 * @brief Returns the intersection of two unordered sets.
 * 
 * @param A First set.
 * @param B Second set.
 * @return std::unordered_set<int> The intersection of A and B.
 */
inline std::unordered_set<int> setIntersection(const std::unordered_set<int>& A,
                                               const std::unordered_set<int>& B) {
    std::unordered_set<int> result;
    for (const auto& x : A) {
        if (B.find(x) != B.end()) {
            result.insert(x);
        }
    }
    return result;
}

/**
 * @brief Returns the difference A \ B.
 * 
 * @param A First set (minuend).
 * @param B Second set (subtrahend).
 * @return std::unordered_set<int> The difference A \ B.
 */
inline std::unordered_set<int> setDifference(const std::unordered_set<int>& A,
                                             const std::unordered_set<int>& B) {
    std::unordered_set<int> result;
    for (const auto& x : A) {
        if (B.find(x) == B.end()) {
            result.insert(x);
        }
    }
    return result;
}
