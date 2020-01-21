#ifndef __COMMON_H__
#define __COMMON_H__

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Returns whether u_set contains item.
 *
 * @tparam T A type for which the hash has been defined.
 * @param u_set The unordered set to search in.
 * @param item The item to search for.
 * @return true item is in u_set.
 * @return false item is not in u_set.
 */
template <typename T>
bool contains(const std::unordered_set<T> &u_set, T item) {
  return u_set.find(item) != u_set.end();
}

/**
 * @brief Returns whether u_map contains key.
 *
 * @tparam T A type for which the hash has been defined.
 * @tparam U Any type.
 * @param u_map The unordered map to search for keys in.
 * @param item The key to search for.
 * @return true Key key is in u_map.
 * @return false Key key is not in u_map.
 */
template <typename T, typename U>
bool contains(const std::unordered_map<T, U> &u_map, T key) {
  return u_map.find(key) != u_map.end();
}

/**
 * @brief Returns the index of item in vec; -1 if item is not found.
 *
 * @tparam T Any type.
 * @param vec The vector to search for item index in.
 * @param item The item to find the index for in vector.
 * @return int The index of item in vec; -1 if item is not found.
 */
template <typename T> int find_in_vector(const std::vector<T> &vec, T item) {
  typename std::vector<T>::const_iterator it;
  it = std::find(vec.begin(), vec.end(), item);
  return it == vec.end() ? -1 : std::distance(vec.begin(), it);
}

/**
 * @brief Returns the unordered set that is the intersection of set_a and set_b.
 *
 * @tparam T A type for which the hash has been defined.
 * @param set_a Set a.
 * @param set_b Set b.
 * @return std::unordered_set<T> The intersection of set_a and set_b, also an
 * unordered_set.
 */
template <typename T>
std::unordered_set<T>
unordered_intersection(const std::unordered_set<T> &set_a,
                       const std::unordered_set<T> &set_b) {
  if (set_a.size() > set_b.size()) {
    return unordered_intersection(set_b, set_a);
  }
  std::unordered_set<T> intersection;
  for (const auto &a : set_a) {
    if (contains(set_b, a)) {
      intersection.insert(a);
    }
  }
  return intersection;
}

/**
 * @brief Returns the unordered set that is the union of set_a and set_b.
 *
 * @tparam T A type for which the hash has been defined.
 * @param set_a Set a.
 * @param set_b Set b.
 * @return std::unordered_set<T> The union of set_a and set_b, also an
 * unordered_set.
 */
template <typename T>
std::unordered_set<T> unordered_union(std::unordered_set<T> set_a,
                                      const std::unordered_set<T> &set_b) {
  set_a.insert(set_b.begin(), set_b.end());
  return set_a;
}

#endif //__COMMON_H__