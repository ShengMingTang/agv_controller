#include "Controller.h"
// tmp version
/* main event loop, loop at rate this->loop_rate */
void Controller::loopOnce()
{   
    auto op_ptr = this->get_joy_signal();
    int16_t op = this->decode_opcode(op_ptr);

    if(!this->priviledged_instr(op) && 
        !(this->stage_bm & MODE_AUTO) && 
        !(this->stage_bm & MODE_AGENT))
    {
        switch(this->mode){
            case MODE_IDLE:
                this->idle(op);
                break;
            case MODE_POS:
                ROS_INFO("Postitioning, please walk around");
                this->idle(op);
                break;
            case MODE_CALIB:
                this->calibration(op);
                break;
            case MODE_TRAINING:
                this->training(op);
                break;
            case MODE_WORKING:
                this->working(op);
                break;
            default:
                ROS_WARN("Undefined mode %d", this->mode);
                break;
        }
    }

    this->drive(this->decode_drive(op_ptr));
    this->monitor_display();
    this->loop_rate.sleep();
}

/* Handling idle mode */
void Controller::idle(const int16_t &_op)
{
    RobotInvoke srv;
    switch(_op){
        case OPCODE_NONE:
            this->work_begin();
            break;
        case OPCODE_CALIB:
            this->calibration(_op);
            break;
        case OPCODE_TRAIN_BEGIN:
            if(!this->train_begin()){
                ROS_ERROR("Train begin failed");
            }
            break;
        case OPCODE_WORK_BEGIN: // work manually, just push Vertex pointers here
            this->add_target(this->nd_target);
            break;
        case OPCODE_AUTO:
            break;
        default:
            ROS_WARN("In Idle mode, %c is not allowed", static_cast<char>(_op));
            break;
    }
}

/* Invoke calibration, block until robot replies */
void Controller::calibration(const int16_t &_op)
{
    auto srv = this->base_driver.invoke(OPCODE_CALIB, vector<int16_t>());
    if(this->base_driver.is_invoke_valid(srv)){
        this->clear();
        this->stage_bm |= MODE_CALIB;
        ROS_INFO("Calibration finished");
    }
    else{
        ROS_ERROR("Calib Error");
    }
    #ifdef ROBOT_CONTROLLER_TEST
        this->mode = MODE_IDLE;
        this->stage_bm |= MODE_CALIB;
    #endif
}

/* Handling training mode */
void Controller::training(const int16_t &_op)
{
    RobotInvoke srv;
    // node
    switch(_op){
        case OPCODE_NONE:
            break;
        case OPCODE_SETNODE:
            if(!this->set_node()){
                ROS_ERROR("Direct SetNode failed");
            }
            break;
        case OPCODE_TRAIN_FINISH:
            if(!this->train_finish()){
                ROS_ERROR("Train finish failed");
            }
            break;
        default:
            ROS_INFO("In Training mode, %c is not allowed", (char)_op);
            break;
    }
}

/* 
    Press A in working mode will flush all works
    
    Polling tracking status
    return true then finish working or not in working, escape from action
    false otherwise and keep trying
*/
bool Controller::working(const int16_t &_op)
{
    this->work_mutex.lock();
    bool ret = false;

    #ifdef ROBOT_CONTROLLER_TEST
        ros::Rate r(1);
        r.sleep();
        this->tracking_status = TRACKING_STATUS_ARRIVAL;
    #endif
    
    if(this->mode != MODE_WORKING){ // finished elsewhere
        ROS_ERROR("mode : %d, Nont in working mode but enter working fn, synchronization issue, Escape", this->mode);
    }
    
    if(_op == OPCODE_WORK_FINISH && !(this->stage_bm & MODE_AUTO)){
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv) && !this->work_list.empty()){
            
            this->ocp_vptr = this->work_list.front();
            this->pose_tracer.set_coor(this->ocp_vptr->pos.x, this->ocp_vptr->pos.y);
            this->work_list.clear();

            #ifdef ROBOT_CONTROLLER_TEST
                this->tracking_status = TRACKING_STATUS_NORMAL;
                this->mode = MODE_IDLE;
            #endif
            
            ROS_INFO("Finish Working manually");
            ret = true;
        }
        else{
            ROS_INFO("Finish Working manually failed, nothing changed");
        }
    }
    else if(this->tracking_status == TRACKING_STATUS_ARRIVAL){
        ROS_INFO("Arrival on target");
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv) && !this->work_list.empty()){
            ROS_INFO("Arrival and finish working");
            this->ocp_vptr = this->work_list.front();
            this->pose_tracer.set_coor(this->ocp_vptr->pos.x, this->ocp_vptr->pos.y);
            this->work_list.pop_front();

            #ifdef ROBOT_CONTROLLER_TEST
                this->tracking_status = TRACKING_STATUS_NORMAL;
                this->mode = MODE_IDLE;
            #endif

            ret = true;
        }
        else{
            ROS_ERROR("Arrival but invoke Work finish Error, keep trying");
        }
    }
    this->work_mutex.unlock();
    return ret;
}

/*
    Invoke setnode operation, return true if successfully sey, otherwise false
    do auto merge when nodes are set with 
    distance <= this->close_enought with any other RouteNodes
*/
bool Controller::set_node()
{
    this->train_mutex.lock();

    PrimitiveType nd;
    auto coor = this->pose_tracer.get_coor();
    VertexType *last_ptr = nullptr;
    
    nd.pos.x = coor.x, nd.pos.y = coor.y;
    nd.route = nd.node = -1;
    
    // valid operation check
    if(!(this->mode & MODE_TRAINING)){
        ROS_ERROR("Not in training mode but want to set node");
        this->train_mutex.unlock();
        return false;
    }
    if(this->nd_training.node > 0){ // has a node on this route
        last_ptr = this->rn_img[this->nd_training.route].back();
    }
    // else no last node
    if(last_ptr && dist(last_ptr->aliases.begin()->pos, nd.pos) < this->close_enough){
        ROS_INFO("Set points < CLOSE_ENOUGH, rejected");
        this->train_mutex.unlock();
        return false;
    }

    // body
    bool ret = false;
    if(this->drive({0, 0})){
        auto srv = this->base_driver.invoke(OPCODE_SETNODE, {SETNODE_PASS_EXACT, this->pose_tracer.get_headway()});
        if(this->base_driver.is_invoke_valid(srv)){
            // graph routine
            nd.route = this->nd_training.route, nd.node = srv.response.feedback[0] - 1;
            
            if(nd.node != this->rn_img[nd.route].size()){
                ROS_ERROR("UART-returned node not continuous, fatal error, setnode ignored");
            }
            else{
                // update training parameters
                this->nd_training.node = nd.node;
                // check if there is a close vertex so that they can merge
                VertexType* vptr = nullptr;
                for(auto vec : this->rn_img){
                    for(auto it : vec){
                        if(dist(it->pos, nd.pos) < this->close_enough){
                            vptr = it;
                            // this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
                            break;
                        }
                    }
                    if(vptr)
                        break;
                }
                // make a vertex for the graph, retrieve its reference if it is a new vertex
                if(!vptr){ // need to create a new one
                    VertexType tmp;
                    vptr = this->graph.add_vertex(tmp);
                    vptr->pos = nd.pos;
                }
                // add the new routenode to the vertex
                vptr->aliases.insert(nd);
                this->rn_img[nd.route].push_back(vptr);
                // build forward and backward edge if it is not the first node in the current route
                if(nd.node > 0){
                    // build edge for forward direction, add it to graph
                    EdgeType e;
                    e.src = this->rn_img[nd.route][nd.node - 1];
                    e.dst = vptr;
                    e.w = this->pose_tracer.get_dist();
                    e.walk = this->pose_tracer.get_path();
                    this->graph.add_edge(e);
                    // flip the edge for backwawrd direction, add it to graph
                    swap(e.src, e.dst);
                    e.walk = flip_walk(e.walk);
                    this->graph.add_edge(e);
                }
                // done
                this->ocp_vptr = vptr;
                ROS_INFO("Node #R%d, #N%d @ (%f,%f) set", nd.route, nd.node, nd.pos.x, nd.pos.y);
                ret = true;
            }
        }
    }
    else{
        ROS_ERROR("Drive error in SetNode, setnode not done");
    }
    this->train_mutex.unlock();
    return ret;
}

/*
    push a target node to work list
    clearing work list first then auto expand target into a list of RouteNodes
*/
bool Controller::add_target(const RouteNode &_nd)
{
    this->work_mutex.lock();
    bool ret = false;

    if(this->stage_bm & MODE_TRAINING){
        if(this->is_target_valid(_nd)){
            this->work_list.clear();
            auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_nd.route][_nd.node]).second;
            this->work_list.insert(this->work_list.end(), path.begin(), path.end());
            stringstream ss;
            ss << "Path : ";
            for(auto it = this->work_list.begin(); it != this->work_list.end(); it++){
                RouteNode nd = *((*it)->aliases.begin());
                if(it != this->work_list.begin()){
                    auto it2 = it;
                    it2--;
                    nd = this->get_affinity_vertex(*it2, *it);
                }
                ss << "R" << nd.route << "N" << nd.node << " -> ";
            }
            ROS_INFO("%s", ss.str().c_str());
            ret = true;
        }
        else{
            ROS_ERROR("Target out of range, plz reset");
        }
    }
    else{
        ROS_ERROR("Not trained but want to add target");
    }
    this->work_mutex.unlock();
    return ret;
}

/* Find the RouteNode on the same path as the curret one */
const RouteNode Controller::get_affinity_vertex(const VertexType* _curr, const VertexType* _vptr)
{
    RouteNode ret;
    ret.route = -1, ret.node = -1;
    if(_vptr){
        set<int16_t> s;
        for(auto it : _curr->aliases){
            s.insert(it.route);
        }
        for(auto it : _vptr->aliases){
            if(s.find(it.route) != s.end()){
                ret = it;
                break;
            }
        }
    }
    if(ret.route == -1 && ret.node == -1){
        ROS_ERROR("Can't find a close affinity RouteNode");
    }
    return ret;
}

/*
    Atomic
    Invoke train_begin operation on this->nd_training.route
    return true if successfully invoked, otherwise false
*/
bool Controller::train_begin()
{
    this->train_mutex.lock();
    if(this->stage_bm & MODE_CALIB){
        vector<int16_t> train_args = {this->nd_training.route};
        this->nd_training.node = -1;
        auto srv = this->base_driver.invoke(OPCODE_TRAIN_BEGIN, train_args);
        if(this->base_driver.is_invoke_valid(srv)){
            // flush all data for route nd.training.route
            this->graph.erase_edge(this->rn_img[this->nd_training.route]);
            for(int i = 0; i < this->rn_img[this->nd_training.route].size(); i++){
                RouteNode nd;
                nd.route = this->nd_training.route, nd.node = i;
                this->rn_img[this->nd_training.route][i]->aliases.erase(nd);
            }
            // garbage collection
            for(auto it = this->graph.vertices.begin(); it != this->graph.vertices.end();){
                if(it->aliases.empty()){
                    it = this->graph.vertices.erase(it);
                }
                else{
                    it++;
                }
            }
            this->rn_img[this->nd_training.route].clear();
            this->drive({0, 0});
            this->ocp_vptr = nullptr;
            ROS_WARN("First node is not automatically set!");
            ROS_WARN("Please set first node manually before any movement");

            #ifdef ROBOT_CONTROLLER_TEST
                this->mode = MODE_TRAINING;
                this->stage_bm |= MODE_TRAINING;
            #endif
            this->train_mutex.unlock();
            return true;
        }
    }
    else{
        ROS_ERROR("Not Calibed but Want to enter Training");
    }
    this->train_mutex.unlock();
    return false;
}

/*
    Atomic
    Invoke train_finish operation on this->nd_training.route
    return true if successfully invoked, otherwise false
*/
bool Controller::train_finish()
{
    this->train_mutex.lock();

    bool ret = false;
    if(this->rn_img[this->nd_training.route].size() >= TRAIN_NODE_MIN){
        auto srv = this->base_driver.invoke(OPCODE_TRAIN_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            #ifdef ROBOT_CONTROLLER_TEST
                this->mode = MODE_IDLE;
            #endif
            ROS_WARN("Once Trained, lock motor if not in tranining mode !");
            ROS_WARN("Note that this piece of info only appears once at the end of every training");
            this->nd_training.route = (this->nd_training.route + 1) % TRAIN_ROUTE_MAX;
            this->nd_training.node = -1;

            auto s = this->dumps_graph();
            ROS_WARN("\n%s", s.c_str());
            ret = true;
        }
        else{
            ROS_ERROR("SetNode Failed when finishing training");
            ROS_ERROR("Train finish operation rejected");
        }
    }
    else{
        ROS_INFO("Should set at least 2 nodes before finishing training, invokation rejected");
    }
    this->train_mutex.unlock();
    return ret;
}

/*  Atomic
    trigger working if work_list is not empty 
    return true if the robot has entered working
    false otherwise
*/
bool Controller::work_begin()
{
    this->work_mutex.lock();
    
    bool ret = false;
    while(!this->work_list.empty() && this->ocp_vptr == this->work_list.front()){
        this->work_list.pop_front();
        ROS_WARN("Same Vertex, omitted");
    }
    if(this->stage_bm & MODE_TRAINING && !this->work_list.empty()){
        // check if there were jobs to do
        VertexType *next_vptr = this->work_list.front();
        // if none, enter working mode and claim.
        if(!(this->is_target_ocp(next_vptr))){
            RouteNode next_nd = this->get_affinity_vertex(this->ocp_vptr, next_vptr);
            vector<int16_t> work_args = {next_nd.route, next_nd.node};
            auto srv = this->base_driver.invoke(OPCODE_WORK_BEGIN, work_args);
            if(this->base_driver.is_invoke_valid(srv)){
                this->tracking_status = TRACKING_STATUS_OUT;// spaghetti
                this->ocp_vptr = this->work_list.front(); // claim
                ROS_INFO("Claim the target node is occupied by the current robot");
                ROS_INFO("Enter working mode");
                #ifdef ROBOT_CONTROLLER_TEST
                    this->mode = MODE_WORKING;
                    this->stage_bm |= MODE_WORKING;
                #endif
                ret = true;
            }
            else if(srv.response.error_code == ERROCODE_EMPTY_ROUTE){
                if(!this->work_list.empty()){
                    this->ocp_vptr = this->work_list.front();
                    this->work_list.pop_front();
                    ROS_ERROR("[Fatal Error] Robot gives empty route, forced give up this node, skip to next one");// @@
                }
            }
        }
        else{
            ROS_INFO("Want to work but target is occupied, waiting");
        }
    }
    this->work_mutex.unlock();
    return ret;
}