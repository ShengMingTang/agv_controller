#include <ros/ros.h>
#include <vector>
#include <deque>
#include "pybot.h"
#include "Proc.h"
class ProcTodo
{
public:
    ProcTodo():
    todos{vector< deque< Ptr<Proc> > >(10)}
    {}
    ~ProcTodo() {}
    void push(Ptr<Proc> _ptr);
    void flush();
    Ptr<Proc> get_proc();
    // void pop();
private:
    vector< deque< Ptr<Proc> > > todos;
};