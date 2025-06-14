/*
 * This file is part of HGVisualizer.
 *
 * Copyright (C) Joe Mruz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include <vector>
#include <optional>
#include <stdexcept>

// This module provides a generational index array implementation.
// It allows for efficient allocation and deallocation of indices, while maintaining the ability to track the generation of each index.
// This is useful for scenarios where entities are frequently added and removed, such as in game development or simulation applications.
namespace allocator {

// A generational index that keeps track of an index, its generation, and whether it is alive or dead.
// The index is used to access an entry in an array, the generation is used to distinguish between different versions of the same index,
// and the alive flag indicates whether the entry is currently valid or has been removed.
class GenerationalIndex {
public:
    // Default constructor creates an invalid index
    GenerationalIndex() : index_(0), generation_(0), alive_(false) {}
    GenerationalIndex(size_t index, size_t generation=0, bool alive=true) : index_(index), generation_(generation), alive_(alive) {}
    
    bool      alive()         const { return alive_; }
    size_t    index()         const { return index_; }
    size_t    generation()    const { return generation_; }

    GenerationalIndex to_alive(bool alive=true) const {
        return GenerationalIndex {index_, generation_, alive_};
    }
    
    GenerationalIndex to_next_generation() const {
        return GenerationalIndex {index_, generation_ + 1, false};
    }

    bool operator==(const GenerationalIndex& other) const {
        return index_ == other.index_ && generation_ == other.generation_ && alive_ == other.alive_;
    }

    bool operator!=(const GenerationalIndex& other) const {
        return !(*this == other);
    }

private:
    size_t    index_;
    size_t    generation_;
    bool      alive_;
};

// A generational entry that holds a value and its associated generational index.
template<typename T>
class GenerationalEntry {
public:
    GenerationalEntry(T value, GenerationalIndex index) : value_(value), index_(index) {}

    T&                  value() { return value_; }
    const T&            value() const { return value_; }
    GenerationalIndex   index() const { return index_; }
    void                increment_generation() {index_ = index_.to_next_generation();}

private:
    T                   value_;
    GenerationalIndex   index_;
};

// A generational index array that manages a collection of generational entries.
// It allows adding, getting, removing, and setting entries by their generational index.
template <typename T>
class GenerationalIndexArray {
public:
    GenerationalIndexArray() : entries_(), free_(), size_(0) {}
    
    // Add a new entry and return its index
    // If there are free indices, reuse the last one
    // Otherwise, create a new entry
    // Note: This function does not check for duplicates
    GenerationalIndex add(const T& value) {
        size_++;
        if (!free_.empty()) {
            auto index = free_.back();
            free_.pop_back();
            entries_[index.index()] = GenerationalEntry<T> {value, index.to_alive()};
            return index;
        } else {
            entries_.emplace_back(GenerationalEntry<T> {value, GenerationalIndex {entries_.size()}});
            return entries_.back().index();
        }
    }

    // Get the value of an entry by its index
    // Returns std::nullopt if the index is dead or out of bounds
    // Otherwise, returns the value of the entry
    std::optional<T> get(const GenerationalIndex& index) const {
        if (index.alive() && index.index() < entries_.size()) {
            return entries_[index.index()].value();
        }
        return std::nullopt;
    }

    // Get the value of an entry by its index
    // Throws std::out_of_range if the index is dead or out of bounds
    // Otherwise, returns the value of the entry
    T& operator[] (const GenerationalIndex& index) const {
        std::optional<T> value = get(index);
        if (value.has_value()) {
            return value.value();
        }
        throw std::out_of_range("Index is dead or out of bounds");
    }

    // Remove an entry by its index
    // This marks the entry as dead and increments its generation
    // The entry can be reused later
    bool remove(const GenerationalIndex& index) {
        if (index.alive() && index.index() < entries_.size()) {
            entries_[index.index()].increment_generation();
            free_.push_back(index);
            size_--;
            return true;
        }

        return false; // Index is dead or out of bounds
    }

    // Set the value of an entry by its index
    // If the index is dead or out of bounds, does nothing
    // Otherwise, updates the value of the entry
    void set(const GenerationalIndex& index, const T& value) {
        if (index.alive() && index.index() < entries_.size()) {
            entries_[index.index()] = GenerationalEntry<T> {value, index};
        }
    }

    // Clear all entries
    void clear() {
        entries_.clear();
        free_.clear();
        size_ = 0;
    }

    // Get the size of the array (number of alive entries)
    // This does not count dead entries or free indices
    size_t size() const { return size_; }

    // Get the capacity of the array (total number of entries, including dead and free indices)
    // This is the size of the entries vector, which may be larger than the number of alive entries
    size_t capacity() const { return entries_.size(); }

    // Iterator for alive entries
    class Iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = GenerationalEntry<T>;
        using difference_type = std::ptrdiff_t;
        using pointer = GenerationalEntry<T>*;
        using reference = GenerationalEntry<T>&;

        Iterator(typename std::vector<GenerationalEntry<T>>::iterator current,
                 typename std::vector<GenerationalEntry<T>>::iterator end,
                 GenerationalIndexArray<T>* parent,
                 bool alive_only = true)
            : current_(current), end_(end), parent_(parent), alive_only_(alive_only) {
            if (alive_only_) {
                advance_to_alive();
            }
        }

        reference operator*() const { return *current_; }
        pointer operator->() const { return &(current_); }

        Iterator& operator++() {
            ++current_;
            if (alive_only_) {
                advance_to_alive();
            }
            return *this;
        }

        bool operator==(const Iterator& other) const {
            return current_ == other.current_;
        }
        bool operator!=(const Iterator& other) const {
            return !(*this == other);
        }
        reference operator[] (size_t offset) const {
            auto it = current_;
            for (size_t i = 0; i < offset && it != end_; ++i) {
                ++it;
                if (alive_only_) {
                    while (it != end_ && !it->index().alive()) {
                        ++it;
                    }
                }
            }
            if (it != end_) {
                return *it;
            }
            throw std::out_of_range("Iterator offset out of range");
        }

        // Erase the current entry and advance to the next alive entry
        // This modifies the parent array by removing the current entry
        // It does not invalidate the iterator, but it may change the current entry
        void erase() {
            if (current_ != end_) {
                if (current_->index().alive()) {
                    parent_->remove(current_->index());
                }
                if (alive_only_) {
                    advance_to_alive();
                } else {
                    ++current_;
                }
            }
        }

    private:
        void advance_to_alive() {
            while (current_ != end_ && !current_->index().alive()) {
                ++current_;
            }
        }
        typename std::vector<GenerationalEntry<T>>::iterator current_;
        typename std::vector<GenerationalEntry<T>>::iterator end_;
        GenerationalIndexArray<T>* parent_;
        bool alive_only_; 
    };

    // Const iterator for alive entries
    class ConstIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = GenerationalEntry<T>;
        using difference_type = std::ptrdiff_t;
        using pointer = const GenerationalEntry<T>*;
        using reference = const GenerationalEntry<T>&;

        ConstIterator(typename std::vector<GenerationalEntry<T>>::const_iterator current,
                      typename std::vector<GenerationalEntry<T>>::const_iterator end,
                      bool alive_only = true)
            : current_(current), end_(end), alive_only_(alive_only) {
            if (alive_only_) {
                advance_to_alive();
            }
        }

        reference operator*() const { return *current_; }
        pointer operator->() const { return &(*current_); }

        ConstIterator& operator++() {
            ++current_;
            if (alive_only_) {
                advance_to_alive();
            }
            return *this;
        }

        bool operator==(const ConstIterator& other) const {
            return current_ == other.current_;
        }
        bool operator!=(const ConstIterator& other) const {
            return !(*this == other);
        }
        reference operator[] (size_t offset) const {
            auto it = current_;
            for (size_t i = 0; i < offset && it != end_; ++i) {
                ++it;
                if (alive_only_) {
                    while (it != end_ && !it->index().alive()) {
                        ++it;
                    }
                }
            }
            if (it != end_) {
                return *it;
            }
            throw std::out_of_range("Iterator offset out of range");
        }

    private:
        void advance_to_alive() {
            while (current_ != end_ && !current_->index().alive()) {
                ++current_;
            }
        }
        typename std::vector<GenerationalEntry<T>>::const_iterator current_;
        typename std::vector<GenerationalEntry<T>>::const_iterator end_;
        bool alive_only_;
    };

    Iterator begin() {return Iterator {entries_.begin(), entries_.end(), this};}
    Iterator end() {return Iterator {entries_.end(), entries_.end(), this};}
    ConstIterator begin() const { return ConstIterator{entries_.cbegin(), entries_.cend()}; }
    ConstIterator end() const { return ConstIterator{entries_.cend(), entries_.cend()}; }
    Iterator begin_all() {return Iterator {entries_.begin(), entries_.end(), this, false};}
    Iterator end_all() {return Iterator {entries_.end(), entries_.end(), this, false};}
    ConstIterator begin_all() const { return ConstIterator{entries_.cbegin(), entries_.cend(), false}; }
    ConstIterator end_all() const { return ConstIterator{entries_.cend(), entries_.cend(), false}; }

private:
    std::vector<GenerationalEntry<T>> entries_;
    std::vector<GenerationalIndex>    free_;
    size_t                            size_;
};

}

#endif