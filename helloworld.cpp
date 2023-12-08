#include <iostream>
#include "include/geometry.h"
// #include "include/draw_geometry.h"
#include <utility>
#include <set>
#include <string>
#include <vector>
#include <iterator>

class Elem
{
public:
    std::string m_name;
    int m_value;

    Elem(std::string name, int value) : m_name(name), m_value(value){};
    int key()
    {
        return m_value * m_value;
    }
};

struct Comp
{
    bool operator()(Elem *left, Elem *right)
    {
        return left->key() < right->key();
    }
};

class ElemList
{
public:
    std::vector<Elem> m_store;
    std::set<Elem *, Comp> m_set;
    ElemList() {
        m_store.push_back(Elem("A", 1));
        m_store.push_back(Elem("B", 2));
        m_store.push_back(Elem("C", 3));
        m_store.push_back(Elem("D", 4));
        for (auto it=m_store.begin(); it!=m_store.end(); ++it) {
            m_set.insert(&(*it));
        }
    };
    void printInorder() {
        for (auto it=m_set.begin(); it!=m_set.end(); ++it) {
            std::cout<<"("<<(*it)->m_name<<"["<<(*it)->key()<<"])|";
        }
        std::cout<<std::endl;
    }
    void changeKeyTest() {
        m_store[1].m_value=7;
        m_store[2].m_value=9;
        printInorder();
        m_set.erase(&m_store[1]);
        std::cout<<"After erase"<<std::endl;
        printInorder();
        std::cout<<"After insert"<<std::endl;
        m_set.insert(&m_store[1]);
        printInorder();
        std::cout<<(m_set.find(&m_store[2]) != m_set.end() )<< std::endl;
    }
};

void start()
{
    ElemList e;
    e.printInorder();
    e.changeKeyTest();
}

int main()
{
    std::cout << "Hello World" << std::endl;
   // start();
    std::string filename="testdata2.txt";
    PlaneSweepInput pi(filename);
}