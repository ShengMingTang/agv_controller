#include "ProcToDo.h"

void ProcTodo::push(Ptr<Proc> _ptr)
{
    if(_ptr->push_dir == PUSH_FRONT){
        this->todos[_ptr->prior].push_front(_ptr);
    }
    else{
        this->todos[_ptr->prior].push_back(_ptr);
    }
}
void ProcToDo::flush()
{
    this->todos = vector< deque< Ptr<Proc> > >(10);
}
Ptr<Proc> ProcTodo::get_proc()
{
    // can be done better if using list
    for(int i = 0; i < this->todos.size(); i++){
        while(!this->todos[i].empty()){
            auto top = this->todos[i].front();
            if(top->is_finished){
                this->todos[i].pop_front();
            }
            else{ // got something to do
                return this->todos[i].front();
            }
        }
    }
    return nullptr;
}