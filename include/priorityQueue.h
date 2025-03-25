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
  PriorityQueue(int size) {
    this->size = size;
    elements = (T *)malloc(size * sizeof(T));
    priorities = (double *)malloc(size * sizeof(double));
  }
  /**
   * @brief Destructor
   */
  ~PriorityQueue() {
    free(elements);
    free(priorities);
  }

  /**
   * @brief Push an element with a priority
   * @param e The element
   * @param p The priority
   */
  void push(T e, double p) {
    if (count < size) {
      elements[count] = e;
      priorities[count] = p;
      count++;
      return;
    }

    for (int i = 0; i < size; i++) {
      if (p < priorities[i]) {
        for (int j = size - 1; j > i; j--) {
          elements[j] = elements[j - 1];
          priorities[j] = priorities[j - 1];
        }
        elements[i] = e;
        priorities[i] = p;
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
      std::cerr << "PriorityQueue is empty" << std::endl;
      exit(1);
    }

    T e = elements[0];
    for (int i = 1; i < size; i++) {
      elements[i - 1] = elements[i];
      priorities[i - 1] = priorities[i];
    }
    count--;
    return e;
  }

  /**
   * @brief Get the element with the lowest priority
   * @return The element
   */
  bool empty() { return count == 0; }

private:
  T *elements;
  double *priorities;
  int size;
  int count;
};
