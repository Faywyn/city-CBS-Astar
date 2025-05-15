/**
 * @file priorityQueue.h
 * @brief Priority Queue
 *
 * This file contains the declaration of the PriorityQueue class. This class is a simple
 * priority queue implementation. With a fixed size, it will keep the elements sorted by
 * their priority. The elements with the lowest priority will be at the front of the queue.
 */
#pragma once
#include <iostream>
#include <spdlog/spdlog.h>
#include <vector>

/**
 * @brief Priority Queue
 * @tparam T Type of the elements
 *
 * This class is a simple priority queue implementation. With a fixed size, it
 * will keep the elements sorted by their priority. The elements with the lowest
 * priority will be at the front of the queue.
 */
template <class T> class PriorityQueue {
public:
  /**
   * @brief Constructor
   * @param size The size of the queue
   */
  PriorityQueue(int size) : size(size), count(0), elements(size) {}

  /**
   * @brief Destructor
   */
  ~PriorityQueue() = default;

  /**
   * @brief Push an element with a priority
   * @param e The element
   * @param p The priority
   */
  void push(T e) {
    // If there is still space, add the element at the end.
    if (count < size) {
      elements[count] = e;
      count++;
      return;
    }

    // Otherwise, find the correct position for the new element
    // so that the elements remain sorted by priority.
    for (int i = 0; i < size; i++) {
      if (e < elements[i]) {
        // Shift elements to make room for the new element.
        for (int j = size - 1; j > i; j--) {
          elements[j] = elements[j - 1];
        }
        elements[i] = e;
        return;
      }
    }
  }

  /**
   * @brief Pop the element with the lowest priority
   * @return The element
   */
  T pop() {
    if (count == 0) {
      spdlog::error("Trying to pop from an empty priority queue");
      return T();
    }

    T e = elements[0];
    // Shift elements to fill the gap left by the removed element.
    for (int i = 1; i < count; i++) {
      elements[i - 1] = elements[i];
    }
    count--;
    return e;
  }

  /**
   * @brief Return if the queue is empty
   * @return True if the queue is empty
   */
  bool empty() { return count == 0; }

private:
  std::vector<T> elements; // Vector to store the elements.
  int size;                // Maximum number of elements.
  int count;               // Current number of elements.
};
