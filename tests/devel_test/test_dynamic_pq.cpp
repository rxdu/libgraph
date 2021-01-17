#include <vector>
#include <cstdint>
#include <iostream>

#include "graph/details/dynamic_priority_queue.hpp"

using namespace rdu;

struct TestItem
{
    TestItem(int64_t id) : id_(id) {}
    TestItem(int64_t id, double val) : id_(id), value_(val) {}

    int64_t id_;
    double value_;
};

int main(int argc, char **argv)
{
    std::cout << "queue" << std::endl;

    std::vector<TestItem *> items;

    DynamicPriorityQueue<TestItem *> q;

    for (int i = 0; i < 5; ++i)
    {
        items.push_back(new TestItem(i, i));
        q.push(items[i]);
    }

    q.print_queue();
    q.make();

    std::cout << "----------" << std::endl;

    items[0]->value_ = 5.0;
    q.push(items[0]);

    q.print_queue();

    return 0;
}