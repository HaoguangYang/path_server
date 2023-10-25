/**
 * @file type_map.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A data structure that maps a type string to an object instance.
 * @version 0.1
 * @date 2023-08-29
 * 
 * @copyright Copyright (c) 2022-2023 Haoguang Yang
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 * 
 */

#ifndef TYPE_MAP_HPP_
#define TYPE_MAP_HPP_

#include <string_view>
#include <unordered_map>
#include <vector>

/**
 * @brief Hash map that supports indexing by insertion order. This implementation is more
 * index-friendly than key lookup-friendly.
 *
 * @tparam ValueType value type to be sotred in the hash map.
 */
template <class ValueType>
class TypeMap {
  typedef std::unordered_map<std::string_view, size_t> InternalMap;
  typedef std::vector<ValueType> InternalStorage;

 public:
  typedef typename InternalStorage::iterator iterator;
  typedef typename InternalStorage::const_iterator const_iterator;
  typedef typename InternalStorage::value_type value_type;

  // map iterators to become value iterators
  const_iterator begin() const { return stor_.begin(); }
  const_iterator end() const { return stor_.end(); }
  iterator begin() { return stor_.begin(); }
  iterator end() { return stor_.end(); }

  // Finds the value associated with the type string in the type map.
  iterator find(const std::string_view& keyStr) {
    auto it = map_.find(keyStr);
    return it == map_.end() ? stor_.end() : (stor_.begin() + it->second);
  }

  iterator find(const std::string& keyStr) {
    auto it = map_.find(keyStr);
    return it == map_.end() ? stor_.end() : (stor_.begin() + it->second);
  }

  // Same as above, const version
  const_iterator find(const std::string_view& keyStr) const {
    auto it = map_.find(keyStr);
    return it == map_.end() ? stor_.cend() : (stor_.cbegin() + it->second);
  }

  const_iterator find(const std::string& keyStr) const {
    auto it = map_.find(keyStr);
    return it == map_.end() ? stor_.cend() : (stor_.cbegin() + it->second);
  }

  // Associates the index value with the key string, and append value storage
  void put(const std::string_view& keyStr, ValueType&& value) {
    map_[keyStr] = stor_.size();
    stor_.emplace_back(std::forward<ValueType>(value));
  }

  void put(const std::string& keyStr, ValueType&& value) {
    map_[std::string_view(keyStr)] = stor_.size();
    stor_.emplace_back(std::forward<ValueType>(value));
  }

  // map indexing and size methods to value storage
  size_t size() const { return stor_.size(); }

  ValueType& operator[](const size_t& ind) { return stor_[ind]; }

  const ValueType& operator[](const size_t& ind) const { return stor_[ind]; }

 private:
  InternalStorage stor_;
  InternalMap map_;
};

#endif  // TYPE_MAP_HPP_
