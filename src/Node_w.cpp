#include "Node_w.h"
template<typename T>
double dist(list<T>& _path)
{
    double ret = 0;
    for(auto it : _path)
        ret += sqrt(it.x * it.x + it.y * it.y);
    return ret;
}

template<typename T>
double dist(const T &_a, const T &_b)
{
    double del_x = (_a.x) - (_b.x), del_y = (_a.y - _b.y);
    return del_x * del_x + del_y * del_y;
}

template<typename T>
Node_w<T>::Node_w(const T &_base)
{
    this->point.x = this->point.y = _base.x = _base.y;
    this->point.z = 0;
    this->aliases.push_back(_base);
}

template<typename T>
void Node_w<T>::add_edge(Edge<T> _e) // create an edge to _adj node_wrapper
{
    this->edges.push_back(_e);
}

template<typename T>
void Node_w<T>::add_alias(const T &_added)
{
    if(!this->is_alias(_added))
        this->alias.push_back(_added);
}

template<typename T>
bool Node_w<T>::is_alias(const T &_alias)const
{
    for(auto it : this->aliases){
        if(*it == _alias){
            return true;
        }
    }
    return false;
}
template<typename T>
bool Node_w<T>::is_close(const T &_other)const
{
    return dist(this->point, _other) <= CLOSE_THRES;
}