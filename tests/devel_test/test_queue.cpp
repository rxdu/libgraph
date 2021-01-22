/*
 * test_queue.cpp
 *
 * Created on: Jan 19, 2021 21:56
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "graph/details/dynamic_priority_queue.hpp"

using namespace rdu;

struct TestElement {
  TestElement() = default;
  TestElement(uint64_t _id, double _value) : id(_id), value(_value){};

  int64_t id;
  double value;
};

struct TEComparator {
  bool operator()(const TestElement& x, const TestElement& y) const {
    if (x.value < y.value) return true;
    return false;
  }
};

std::ostream& operator<<(std::ostream& os, const TestElement& e) {
  os << e.value;
  return os;
}

int main(int argc, char* argv[]) {
  DynamicPriorityQueue<TestElement, TEComparator> queue(5);
  std::vector<TestElement> elements;

  std::size_t i = 0;
  elements.push_back(TestElement(i++, 10));
  elements.push_back(TestElement(i++, 5));
  elements.push_back(TestElement(i++, 20));
  elements.push_back(TestElement(i++, 8));
  elements.push_back(TestElement(i++, 6));
  elements.push_back(TestElement(i++, 2));
  elements.push_back(TestElement(i++, 1));

  for (const auto& element : elements) {
    queue.Push(element);
  }

  queue.PrintQueue();

  elements[6].value = 3;
  queue.Update(elements[6]);

  queue.PrintQueue();

  elements[5].value = 7;
  queue.Update(elements[5]);

  queue.PrintQueue();

  std::cout << "Peeking: " << queue.Peek() << std::endl;

  queue.PrintQueue();

  std::cout << "Popping: " << queue.Pop() << std::endl;

  queue.PrintQueue();

  queue.Pop();

  queue.PrintQueue();

  return 0;
}