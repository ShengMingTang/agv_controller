#include "Controller.h"

extern list<WalkUnitType> flip_walk(const list<WalkUnitType> &_walk);
extern double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b);

void Controller::loopOnce()
{
    #if ROBOT_CONTROLLER_TEST
        this->stage_bm |= this->mode;
    #endif
    // loopOnce front
    this->runtime_vars_mgr(RUNTIME_VARS_SET);
    bool any_privi = this->priviledged_instr();

    // loopOnce body (User-Custom space)
    if(!any_privi){
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
}
void Controller::idle()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            if(this->stage_bm & MODE_TRAINING){
                // check if there were jobs to do
                if(!this->work_list.empty()){
                    VertexType *next_vptr = this->work_list.front();
                    bool is_ocp = false;
                    #if ROBOT_CONTROLLER_TEST
                        is_ocp = false;
                    #endif
                    #if ROBOT_CONTROLLER_WIFI
                        // ask if some has occupied it
                        is_ocp = this->is_target_ocp(next_vptr);
                    #endif
                    // if none, enter working mode and claim.
                    if(!is_ocp){
                        RouteNode next_nd = *(next_vptr->aliases.begin());
                        vector<int16_t> work_args = {next_nd.route, next_nd.node};
                        srv = this->base_driver.invoke(Opcode::OPCODE_WORK_BEGIN, work_args);
                        if(this->base_driver.is_invoke_valid(srv)){
                            // claim
                            this->ocp_vptr = this->work_list.front();
                            ROS_INFO("Head for R%dN%d", next_nd.route, next_nd.node);
                        }
                        else{
                            ROS_ERROR("<Working begin Srv-Err>");
                        }
                    }
                }
            }
            break;
        case Opcode::OPCODE_CALIB:
            srv = this->base_driver.invoke(Opcode::OPCODE_CALIB, vector<int16_t>());
            #if ROBOT_CONTROLLER_TEST
                ROS_INFO("Start Calib (Test)");
                this->drive({0, 0});
                ROS_WARN("Forced stop");
                this->mode = MODE_CALIB;
                this->clear();
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Start Calib");
                    this->drive({0, 0});
                    ROS_WARN("Forced stop");
                    this->clear();
                    this->base_driver.invoke(Opcode::OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
                    // test
                    this->stage_bm |= MODE_CALIB;
                }
                else{
                    ROS_ERROR("<Calib Srv-Err>");
                }
            #endif
            break;
        case Opcode::OPCODE_TRAIN_BEGIN:{
            if(this->stage_bm & MODE_CALIB){
                vector<int16_t> train_args = {this->nd_training.route};
                this->nd_training.node = 0;
                srv = this->base_driver.invoke(Opcode::OPCODE_TRAIN_BEGIN, train_args);
                #if ROBOT_CONTROLLER_TEST
                    node_ct = 0;
                    ROS_INFO("Start Training @ R%d", this->nd_training.route);
                    // flush all data for route nd.training.route
                    this->graph.erase(this->rn_img[this->nd_training.route]);
                    this->rn_img[this->nd_training.route].clear();
                    this->drive({0, 0}); // forced stop
                    ROS_WARN("Forced stop");
                    this->mode = MODE_TRAINING;
                    this->stage_bm |= MODE_TRAINING;
                    if(this->set_node()){
                        ROS_WARN("Forced to set first node automatically");
                    }
                    else{
                        ROS_ERROR("Set first node error");
                    }
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        ROS_INFO("Start Training @ R%d", this->nd_training.route);
                        // flush all data for route nd.training.route
                        this->graph.erase(this->rn_img[this->nd_training.route]);
                        this->rn_img[this->nd_training.route].clear();
                        this->drive({0, 0}); // forced stop
                        ROS_WARN("Forced stop");
                        if(this->set_node()){
                            ROS_WARN("Forced to set first node automatically");
                        }
                        else{
                            ROS_ERROR("Set first node error");
                        }
                    }
                    else{
                        ROS_ERROR("<Traing begin Srv-Err>");
                    }
                #endif
            }
            else{
                ROS_ERROR("Not Calibed but Want to enter Training");
            }
            break;
        }
        /* work manually*/
        case Opcode::OPCODE_WORK_BEGIN:
            if(this->stage_bm & MODE_TRAINING){
                if(this->nd_target.node < this->rn_img[this->nd_target.route].size()){
                    this->work_list.clear();
                    auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[this->nd_target.route][this->nd_target.node]).second;
                    this->work_list.insert(this->work_list.end(), path.begin(), path.end());
                    for(auto it : this->work_list){
                        RouteNode nd = *(it->aliases.begin());
                        ROS_INFO("(( %d, %d ))", nd.route, nd.node);
                    }

                }
                else
                    ROS_ERROR("Target out of range, plz reset");
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
    #else
        ROS_INFO("In Calibration");
    #endif
}
void Controller::training()
{
    RobotInvoke srv;
    // node
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_SETNODE:
            if(!this->set_node()){
                ROS_ERROR("Direct SetNode failed (Controller report)");
            }
            break;
        case Opcode::OPCODE_TRAIN_FINISH:
            if(this->rn_img[this->nd_training.route].size() >= TRAIN_NODE_MIN){
                #if ROBOT_CONTROLLER_TEST
                    if(this->set_node()){
                        ROS_WARN("Forced stop, one node automatically set");
                    }
                    else{
                        ROS_ERROR("SetNode Failed when finishing training (Controller report)");
                    }
                    this->mode = MODE_IDLE;
                    ROS_WARN("Training finished");
                    ROS_WARN("Once Trained, lock motor if not in tranining mode !");
                    ROS_WARN("Note that this piece of info only appears once at the end of every training");
                    this->nd_training.route = (this->nd_training.route + 1) % TRAIN_ROUTE_MAX; 
                    this->nd_training.node = 0;
                    // test
                    auto s = this->dumps_graph();
                    ROS_WARN("\n%s", s.c_str());
                #else
                    if(this->set_node()){
                        ROS_WARN("Forced stop, one node automatically set");
                    }
                    else{
                        ROS_ERROR("SetNode Failed when finishing training (Controller report)");
                        ROS_ERROR("SetNode failed but keep doing Training-Finish");
                    }
                    srv = this->base_driver.invoke(Opcode::OPCODE_TRAIN_FINISH, vector<int16_t>());
                    if(this->base_driver.is_invoke_valid(srv)){
                        ROS_WARN("Training finished");
                        ROS_WARN("Once Trained, lock motor if not in tranining mode !");
                        ROS_WARN("Note that this piece of info only appears once at the end of every training");
                        this->nd_training.route = (this->nd_training.route + 1) % TRAIN_ROUTE_MAX;
                        this->nd_training.node = 0;
                        // test
                        auto s = this->dumps_graph();
                        ROS_WARN("\n%s", s.c_str());
                    }
                    else{
                        ROS_ERROR("<Traing finished Srv-Err>");
                    }
                #endif
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
    if(this->tracking_status == Tracking_status::TRACKING_STATUS_ARRIVAL){
        auto srv = this->base_driver.invoke(Opcode::OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            this->work_list.pop_front();
            // claim that we have given up that occupied node
            this->ocp_vptr = this->work_list.front();
            auto coor = this->target_vptr->aliases.begin();
            this->pose_tracer.set_coor(coor->pos.x, coor->pos.y);
            ROS_INFO("Work finished");
        }
        else{
            ROS_ERROR("<Working finish Srv-Err>");
        }
    }
    else if(!this->check_safety()){ // not safe, stop current work, call help
        auto srv = this->base_driver.invoke(Opcode::OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            ROS_WARN("Encounter an obstacle in front");
            this->isr(ISR_OBSTACLE);
        }
        else{
            ROS_ERROR("<Working finish Srv-Err>");
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
    if(last_ptr && dist(last_ptr->aliases.begin()->pos, nd.pos) < CLOSE_ENOUGH){
        ROS_INFO("Set points < CLOSE_ENOUGH, rejected");
        return false;
    }
    // body
    if(this->drive({0, 0})){ // forced stop
        ROS_WARN("Forced stop");
        auto srv = this->base_driver.invoke(Opcode::OPCODE_SETNODE, {SETNODE_PASS_EXACT, this->pose_tracer.get_headway()});
        #if ROBOT_CONTROLLER_TEST
            // modify response
            srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
            srv.response.error_code = (int16_t)Errcode::ERRCODE_OK; // ok
            srv.response.feedback = vector<int16_t>(1, node_ct++);
        #endif
        if(this->base_driver.is_invoke_valid(srv)){
            // graph routine
            nd.route = this->nd_training.route, nd.node = srv.response.feedback[0];
            // update training parameters
            this->nd_training.node = nd.node;
            // check if there is a close vertex so that they can merge
            VertexType* vptr = nullptr;
            for(auto vec : this->rn_img){
                for(auto it : vec){
                    if(dist(it->pos, nd.pos) < CLOSE_ENOUGH){
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
        else{
            ROS_ERROR("<SetNode Srv-Err>, this call has no effect");
            return false;
        }

    }
    else{
        ROS_ERROR("Drive error in SetNode");
        return false;
    }
}