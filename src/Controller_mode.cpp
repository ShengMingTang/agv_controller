#include "Controller.h"

extern list<WalkUnitType> flip_walk(const list<WalkUnitType> &_walk);
extern double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b);

void Controller::loopOnce()
{
    // loopOnce front
    this->runtime_vars_mgr(RUNTIME_VARS_SET);
    bool any_privi = this->priviledged_instr();

    // loopOnce body (User-Custom space)
    if(!any_privi){
        switch(this->mode){
            case Mode::MODE_IDLE:
                this->idle();
                break;
            case Mode::MODE_HOMING:
                this->homing();
                break;
            case Mode::MODE_TRAINING:
                if(this->stage_bm & STAGE_CALIBED)
                    this->training();
                else{
                    ROS_WARN("Not Homed but want to enter Trainng mode");
                    ROS_WARN("Direct forced switch to Idle mode");
                    this->mode = Mode::MODE_IDLE;
                }
                break;
            case Mode::MODE_WORKING:
                if(this->stage_bm & STAGE_TRAINED)
                    this->working();
                else{
                    ROS_WARN("Not Trained but want to enter Working mode");
                    this->mode = Mode::MODE_IDLE;
                }
                break;
            default:
                ROS_WARN("Undefined mode");
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
            if(this->stage_bm & STAGE_TRAINED){
                // check if there were jobs to do
                if(!this->work_list.empty()){
                    VertexType *next_vptr = this->work_list.front();
                    bool is_ocp = false;
                    #if ROBOT_CONTROLLER_WIFI
                        // ask if some has occupied it
                        is_ocp = this->is_target_ocp(next_vptr);
                    #endif
                    // if none, enter working mode and claim.
                    if(!is_ocp){
                        RouteNode next_nd = *(next_vptr->aliases.begin());
                        vector<int16_t> work_args = {next_nd.route, next_nd.node};
                        srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, work_args);
                        if(this->base_driver.is_invoke_valid(srv)){
                            // claim
                            this->ocp_vptr = this->work_list.front();
                            ROS_INFO("Enter Working mode, heading for R%dN%d", next_nd.route, next_nd.node);
                        }
                        else{
                            ROS_ERROR("<Working begin Srv-Err>");
                        }
                    }
                }
            }
            break;
        case Opcode::OPCODE_HOMEING:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_HOMEING, vector<int16_t>());
            #if ROBOT_CONTROLLER_TEST
                this->mode = Mode::MODE_HOMING;
                ROS_INFO("Start Homing");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->clear();
                    ROS_INFO("Start Homing");
                }
                else{
                    ROS_ERROR("<Homing Srv-Err>");
                }
            #endif
            break;
        case Opcode::OPCODE_TRAIN_BEGIN:{
            vector<int16_t> train_args = {this->nd_training.route};
            this->nd_training.node = 0;
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_BEGIN, train_args);
            #if ROBOT_CONTROLLER_TEST
                this->mode = Mode::MODE_TRAINING;
                node_ct = 0;
                this->nd_training.route = route_ct++;
                ROS_INFO("Start Training @ R%d", this->nd_training.route);
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Start Training @ R%d", this->nd_training.route);
                }
                else{
                    ROS_ERROR("<Traing begin Srv-Err>");
                }
            #endif
            break;
        }
        case Opcode::OPCODE_WORK_BEGIN:
            /* work manually*/
            this->work_list.push_back(this->rn_img[nd_target.route][nd_target.node]);
        default:
            ROS_WARN("In Idle mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive(this->op_vel);
}
void Controller::homing()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_ORIGIN:
            if( !(this->stage_bm & STAGE_CALIB_BEGIN) ){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_ORIGIN, vector<int16_t>());
                #if ROBOT_CONTROLLER_TEST
                    this->pose_tracer.clear();
                    // this->is_origin_set = true;
                    this->stage_bm |= STAGE_ORIGIN_SET;
                    ROS_INFO("Origin set");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){\
                        this->pose_tracer.clear();
                        // this->is_origin_set = true;
                        this->stage_bm |= STAGE_ORIGIN_SET;
                        ROS_INFO("Origin set");
                    }
                    else{
                        ROS_ERROR("<Origin set Srv-Err>");
                    }
                #endif
            }
            break;
        case Opcode::OPCODE_CALIB_BEGIN:
            if((this->stage_bm & STAGE_ORIGIN_SET) && !(this->stage_bm & STAGE_CALIB_BEGIN)){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_BEGIN, vector<int16_t>());
                #if ROBOT_CONTROLLER_TEST
                    // this->is_calib_begin = true;
                    this->stage_bm |= STAGE_CALIBED;
                    ROS_INFO("Calib begin");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->stage_bm |= STAGE_CALIB_BEGIN;
                        ROS_INFO("Calibration begin");
                    }
                    else{
                        ROS_ERROR("<Calib begin Srv-Err>");
                    }
                #endif
            }
            else{
                if(!(this->stage_bm & STAGE_ORIGIN_SET))
                    ROS_ERROR("[Calib begin Err], Set Origin first plz");
                else if(this->stage_bm & STAGE_CALIB_BEGIN)
                    ROS_ERROR("[Calib begin Err], calib already started");
            }
            break;
        case Opcode::OPCODE_CALIB_FINISH:
            if(this->stage_bm & STAGE_CALIB_BEGIN){
                this->stage_bm |= STAGE_CALIBED;
                this->stage_bm &= ~(STAGE_CALIB_BEGIN);
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_FINISH, vector<int16_t>());
                #if ROBOT_CONTROLLER_TEST
                    ROS_INFO("Calib finish");
                    this->mode = Mode(Mode::MODE_IDLE);
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        ROS_INFO("Calib finish, switch to Idle mode");
                    }
                    else{
                        ROS_ERROR("<Calib_finish Srv-Err>");
                    }
                #endif
            }
            else{
                ROS_ERROR("[Calib finish Err], Calib_begin not executed");
            }
            break;
        default:
            ROS_INFO("In Homing mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive(this->op_vel);
}
void Controller::training()
{
    RobotInvoke srv;
    // node
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_SETNODE:
            this->set_node();
            break;
        case Opcode::OPCODE_TRAIN_FINISH:
            if(this->nd_training.node >= TRAIN_NODE_MIN){
                this->stage_bm |= STAGE_TRAINED;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_FINISH, vector<int16_t>());
                #if ROBOT_CONTROLLER_TEST
                    this->mode = Mode::MODE_IDLE;
                    this->nd_training.route++;
                    ROS_INFO("Training finished");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->nd_training.route++;
                        ROS_INFO("Training finished, training route inc");
                    }
                    else{
                        ROS_ERROR("<Traing finished Srv-Err>");
                    }
                #endif
            }
            else{
                ROS_INFO("Should set at least 2 nodes before finishing training");
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
    if(!this->check_safety()){ // not safe, stop current work, call help
        auto srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            ROS_WARN("Encounter an obstacle in front");
            this->isr(ISR_OBSTACLE);
        }
        else{
            ROS_ERROR("<Working finish Srv-Err>");
        }
    }
    else{
        if(this->tracking_status == Tracking_status::TRACKING_STATUS_ARRIVAL){
            auto srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_FINISH, vector<int16_t>());
            if(this->base_driver.is_invoke_valid(srv)){
                this->work_list.pop_front();
                // claim that we have given up that occupied node
                this->ocp_vptr = this->work_list.front();
                ROS_INFO("Work finished");
            }
            else{
                ROS_ERROR("<Working finish Srv-Err>");
            }
        }
        else{
            ROS_INFO("I'm working happily!");
        }
    }
}
PrimitiveType Controller::set_node()
{
    auto srv = this->base_driver.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
    PrimitiveType nd;
    nd.route = nd.node = -1;
    #if ROBOT_CONTROLLER_TEST
        // modify response
        srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
        srv.response.error_code = (int16_t)Errcode::ERRCODE_OK; // ok
        srv.response.feedback = vector<int16_t>(1, node_ct++);
    #endif
    if(this->base_driver.is_invoke_valid(srv)){
        // graph routine
        nd.route = this->nd_training.route, nd.node = srv.response.feedback[0];
        auto coor = this->pose_tracer.get_coor();
        nd.pos.x = coor.x, nd.pos.y = coor.y;
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
        if(vptr == nullptr){ // need to create a new one
            VertexType tmp;
            vptr = this->graph.add_vertex(tmp);
            vptr->pos = nd.pos;
        }
        // add the new routenode to the vertex
        vptr->aliases.insert(nd);
        this->rn_img[nd.route].push_back(vptr);
        // build forward and backward edge if it is not the first node in the current route
        if(nd.node >= 1){
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
        this->nd_training.node++;
        ROS_INFO("Node #R%d, #N%d @ (%f,%f) set successfully", nd.route, nd.node, nd.pos.x, nd.pos.y);
    }
    else{
        ROS_ERROR("<SetNode Srv-Err>, this call has no effect");
    }
    return nd;
}