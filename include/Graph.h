#ifndef GRAPH_H
#define GRAPH_H

#include <cstdlib>
#include <set>
#include <list>
#include <vector>
#include <map>
using namespace std;

template<typename V, typename E>
class Graph
{
public:
    Graph() {}
    ~Graph() {}
    V* add_vertex(const V &_add);
    /* return true if successfully added*/
    bool add_edge(const E _edge);
    void clear();
    /* return pair of cost and path */
    pair< double, list<V*> > shortest_path(const V *_start, const V *_end);
private: 
    // map< V, list<E> > vertices;
    list<V> vertices;
    map<V*, list<E> > edges;

};

template<typename V, typename E>
V* Graph<V, E>::add_vertex(const V &_add)
{
    this->vertices.push_back(_add);
    V *ptr = &(this->vertices.back());
    this->edges[ptr] = list<E>();
    return ptr;  
}

template<typename V, typename E>
bool Graph<V, E>::add_edge(const E _edge)
{
    if(this->edges.find(_edge.src)!= this->edges.end()){
        this->edges[_edge.src].push_back(_edge);
        return true;
    }
    return false;
}

template<typename V, typename E>
void Graph<V, E>::clear()
{
    this->vertices.clear();
}

template<typename V, typename E>
pair<double, list<V*> > Graph<V, E>::shortest_path(const V *_start, const V *_end)
{
    pair<double, list<V*> > ret;
    return ret;
}

#endif