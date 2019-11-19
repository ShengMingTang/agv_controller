#include "Controller.h"

extern list<WalkUnitType> flip_walk(const list<WalkUnitType> &_walk);
extern double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b);

void Controller::loopOnce()
{
    // loopOnce front
    this->runtime_vars_mgr(RUNTIME_VARS_SET);

    // loopOnce body (User-Custom space)
    if(!this->priviledged_instr()){
        switch(this->mode){
            case MODE_IDLE:
                this->idle();
                break;
            case MODE_POS:
                ROS_INFO("Postitioning, please walk around");
                this->idle();
                break;
            case MODE_CALIB:
                this->calibration();
            case MODE_TRAINING:
                this->training();
                break;
            case MODE_WORKING:
                this->working();
                break;
            default:
                ROS_WARN("Undefined mode %d", this->mode);
                break;
        }
    }

    // loopOnce back
    this->runtime_vars_mgr(RUNTIME_VARS_RESET);
    this->monitor_display();
    this->loop_rate.sleep();
}
void Controller::idle()
{
    RobotInvoke srv;
    switch(this->op){
        case OPCODE_NONE:
            if(this->stage_bm & MODE_TRAINING && !this->work_list.empty()){
                // check if there were jobs to do
                VertexType *next_vptr = this->work_list.front();
                bool is_ocp = false;
                
                // if wifi is on, is_cop got a chance to be true, otherwise false
                is_ocp = this->is_target_ocp(next_vptr);

                // if none, enter working mode and claim.
                if(!is_ocp){
                    RouteNode next_nd = *(next_vptr->aliases.begin());
                    vector<int16_t> work_args = {next_nd.route, next_nd.node};
                    srv = this->base_driver.invoke(OPCODE_WORK_BEGIN, work_args);
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->ocp_vptr = this->work_list.front(); // claim
                    }
                }
            }
            break;
        case OPCODE_CALIB:
            srv = this->base_driver.invoke(OPCODE_CALIB, vector<int16_t>());
            if(this->base_driver.is_invoke_valid(srv)){
                this->drive({0, 0});
                ROS_WARN("Forced stop");
                this->clear();
                this->calibration();
            }
            ROS_INFO("bm = %d", this->stage_bm);
            break;
        case OPCODE_TRAIN_BEGIN:{
            if(this->stage_bm & MODE_CALIB){
                vector<int16_t> train_args = {this->nd_training.route};
                this->nd_training.node = 0;
                srv = this->base_driver.invoke(OPCODE_TRAIN_BEGIN, train_args);
                if(this->base_driver.is_invoke_valid(srv)){
                    // flush all data for route nd.training.route
                    this->graph.erase(this->rn_img[this->nd_training.route]);
                    this->rn_img[this->nd_training.route].clear();
                    this->drive({0, 0}); // forced stop
                    ROS_WARN("Forced stop");

                    #ifdef ROBOT_CONTROLLER_TEST
                        this->mode = MODE_TRAINING;
                        this->stage_bm |= MODE_TRAINING;
                    #endif

                    if(this->set_node()){
                        ROS_WARN("First node automatically");
                    }
                    else{
                        ROS_ERROR("Set first node error");
                    }
                }
            }
            else{
                ROS_INFO("bm = %d", this->stage_bm);
                ROS_ERROR("Not Calibed but Want to enter Training");
            }
            break;
        }
        /* work manually, just push node here*/
        case OPCODE_WORK_BEGIN:
            if(this->stage_bm & MODE_TRAINING){
                if(this->nd_target.node < this->rn_img[this->nd_target.route].size()){
                    this->work_list.clear();
                    auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[this->nd_target.route][this->nd_target.node]).second;
                    this->work_list.insert(this->work_list.end(), path.begin(), path.end());
                    stringstream ss;
                    for(auto it : this->work_list){
                        RouteNode nd = *(it->aliases.begin());
                        ss << "((" << nd.route << ", " << nd.node << "))->";
                    }
                    ROS_INFO("%s", ss.str().c_str());
                }
                else{
                    ROS_ERROR("Target out of range, plz reset");
                }
            }
            break;
        default:
            ROS_WARN("In Idle mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive(this->op_vel);
}

/* busy wait */
void Controller::calibration()
{
    #ifdef ROBOT_CONTROLLER_TEST
        this->mode = MODE_IDLE;
        this->stage_bm |= MODE_CALIB;
    #endif
    ROS_WARN("Direct modification on stage_bm |= MODE_CALIB");
    this->stage_bm |= MODE_CALIB;
}
void Controller::training()
{
    RobotInvoke srv;
    // node
    switch(this->op){
        case OPCODE_NONE:
            break;
        case OPCODE_SETNODE:
            if(!this->set_node()){
                ROS_ERROR("Direct SetNode failed (Controller report)");
            }
            break;
        case OPCODE_TRAIN_FINISH:
            if(this->rn_img[this->nd_training.route].size() >= TRAIN_NODE_MIN){
                if(this->set_node()){
                    ROS_WARN("Forced stop, one node automatically set");
                }
                else{
                    ROS_ERROR("[Controller report] SetNode Failed when finishing training");
                    ROS_ERROR("[Controller report] SetNode failed but keep doing Training-Finish");
                }
                srv = this->base_driver.invoke(OPCODE_TRAIN_FINISH, vector<int16_t>());
                if(this->base_driver.is_invoke_valid(srv)){
                    #ifdef ROBOT_CONTROLLER_TEST
                        this->mode = MODE_IDLE;
                    #endif
                    ROS_WARN("Once Trained, lock motor if not in tranining mode !");
                    ROS_WARN("Note that this piece of info only appears once at the end of every training");
                    this->nd_training.route = (this->nd_training.route + 1) % TRAIN_ROUTE_MAX;
                    this->nd_training.node = 0;
                    // test
                    auto s = this->dumps_graph();
                    ROS_WARN("\n%s", s.c_str());
                }
            }
            else{
                ROS_INFO("Should set at least 2 nodes before finishing training, invokation rejected");
            }
            break;
        default:
            ROS_INFO("In Training mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive(this->op_vel);
}
/* 
    Polling tracking status, if emergency occur then forced exiting working
    All wokring-related data will be kept.
    Working will resume as soon as emergency is removed
*/
void Controller::working()
{
    if(this->tracking_status == TRACKING_STATUS_ARRIVAL){
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            this->work_list.pop_front();
            // claim that we have given up that occupied node
            this->ocp_vptr = this->work_list.front();
            auto coor = this->target_vptr->aliases.begin();
            this->pose_tracer.set_coor(coor->pos.x, coor->pos.y);
        }
    }
    else if(!this->check_safety()){ // not safe, stop current work, call help
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            auto sig_srv = this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_2S, 1});
        }
    }
    else{
        ROS_INFO("I'm working happily!");
    }
}
bool Controller::set_node()
{
    PrimitiveType nd;
    auto coor = this->pose_tracer.get_coor();
    nd.pos.x = coor.x, nd.pos.y = coor.y;
    nd.route = nd.node = -1;
    if(!(this->mode & MODE_TRAINING)){
        ROS_ERROR("Not in training mode but want to set node");
        return false;
    }
    VertexType *last_ptr = nullptr;
    if(this->nd_training.node > 0){ // has last node on this route
        last_ptr = this->rn_img[this->nd_training.route].back();
    }
    // else no last node
    if(last_ptr && dist(last_ptr->aliases.begin()->pos, nd.pos) < this->close_enough){
        ROS_INFO("[Controller] Set points < CLOSE_ENOUGH, rejected");
        return false;
    }
    // body
    if(this->drive({0, 0})){ // forced stop
        ROS_WARN("Forced stop");
        auto srv = this->base_driver.invoke(OPCODE_SETNODE, {SETNODE_PASS_EXACT, this->pose_tracer.get_headway()});
        if(this->base_driver.is_invoke_valid(srv)){
            // graph routine
            nd.route = this->nd_training.route, nd.node = srv.response.feedback[0];
            // update training parameters
            this->nd_training.node = nd.node;
            // check if there is a close vertex so that they can merge
            VertexType* vptr = nullptr;
            for(auto vec : this->rn_img){
                for(auto it : vec){
                    if(dist(it->pos, nd.pos) < this->close_enough){
                        vptr = it;
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
            ROS_INFO("Node #R%d, #N%d @ (%f,%f) set successfully", nd.route, nd.node, nd.pos.x, nd.pos.y);
            return true;
        }
        return false;
    }
    else{
        ROS_ERROR("Drive error in SetNode, setnode not done");
        return false;
    }
}