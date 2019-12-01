#include "Controller.h"

extern list<WalkUnitType> flip_walk(const list<WalkUnitType> &_walk);
extern double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b);

void Controller::loopOnce()
{
    // loopOnce front
    this->runtime_vars_mgr(RUNTIME_VARS_SET);

    // loopOnce body (User-Custom space)
    if(!this->priviledged_instr() && !(this->stage_bm & MODE_AUTO)){
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
                break;
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
            this->trigger_working();
            break;
        case OPCODE_CALIB:
            srv = this->base_driver.invoke(OPCODE_CALIB, vector<int16_t>());
            if(this->base_driver.is_invoke_valid(srv)){
                this->drive({0, 0});
                this->clear();
                this->calibration();
            }
            break;
        case OPCODE_TRAIN_BEGIN:{
            if(this->stage_bm & MODE_CALIB){
                vector<int16_t> train_args = {this->nd_training.route};
                this->nd_training.node = -1;
                srv = this->base_driver.invoke(OPCODE_TRAIN_BEGIN, train_args);
                if(this->base_driver.is_invoke_valid(srv)){
                    // flush all data for route nd.training.route
                    this->graph.erase(this->rn_img[this->nd_training.route]);
                    this->rn_img[this->nd_training.route].clear();
                    this->drive({0, 0});

                    ROS_WARN("First node is not automatically set!");
                    ROS_WARN("Please set first node manually before any movement");

                    #ifdef ROBOT_CONTROLLER_TEST
                        this->mode = MODE_TRAINING;
                        this->stage_bm |= MODE_TRAINING;
                    #endif
                }
            }
            else{
                ROS_ERROR("Not Calibed but Want to enter Training");
            }
            break;
        }
        /* work manually, just push node here*/
        case OPCODE_WORK_BEGIN:
            this->add_target(this->nd_target);
            break;
        case OPCODE_AUTO_BEGIN:
        case OPCODE_AUTO_FINISH:
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
    ROS_WARN("Direct modification of stage_bm |= MODE_CALIB");
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
                ROS_ERROR("Direct SetNode failed");
            }
            break;
        case OPCODE_TRAIN_FINISH:
            if(this->rn_img[this->nd_training.route].size() >= TRAIN_NODE_MIN){
                srv = this->base_driver.invoke(OPCODE_TRAIN_FINISH, vector<int16_t>());
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
                }
                else{
                    ROS_ERROR("SetNode Failed when finishing training");
                    ROS_ERROR("Train finish operation rejected");
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

    return true if working is finished, else false
*/
bool Controller::working()
{
    #ifdef ROBOT_CONTROLLER_TEST
        ros::Rate r(1);
        r.sleep();
        this->tracking_status = TRACKING_STATUS_ARRIVAL;
    #endif
    if(this->tracking_status == TRACKING_STATUS_ARRIVAL){
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            if(!this->work_list.empty()){
                this->ocp_vptr = this->work_list.front();
                this->pose_tracer.set_coor(this->ocp_vptr->pos.x, this->ocp_vptr->pos.y);
                this->work_list.pop_front();
            }

            #ifdef ROBOT_CONTROLLER_TEST
                this->tracking_status = TRACKING_STATUS_NORMAL;
                this->mode = MODE_IDLE;
            #endif

            return true;
        }
    }
    else if(!this->check_safety()){ // not safe, stop current work, call help
        auto srv = this->base_driver.invoke(OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            auto sig_srv = this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
            ROS_INFO("Stop working due to unsafe condition");
        }
    }
    return false;
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
        ROS_INFO("Set points < CLOSE_ENOUGH, rejected");
        return false;
    }
    // body
    if(this->drive({0, 0})){
        auto srv = this->base_driver.invoke(OPCODE_SETNODE, {SETNODE_PASS_EXACT, this->pose_tracer.get_headway()});
        if(this->base_driver.is_invoke_valid(srv)){
            // graph routine
            nd.route = this->nd_training.route, nd.node = srv.response.feedback[0] - 1;
            
            // quick return 
            if(nd.node != this->rn_img[nd.route].size()){
                ROS_ERROR("UART-returned node not continuous, fatal error, setnode ignored");
                return false;
            }
            // update training parameters
            this->nd_training.node = nd.node;
            // check if there is a close vertex so that they can merge
            VertexType* vptr = nullptr;
            for(auto vec : this->rn_img){
                for(auto it : vec){
                    if(dist(it->pos, nd.pos) < this->close_enough){
                        vptr = it;
                        this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
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
            return true;
        }
        return false;
    }
    else{
        ROS_ERROR("Drive error in SetNode, setnode not done");
        return false;
    }
}
bool Controller::add_target(const RouteNode &_nd)
{
    if(this->stage_bm & MODE_TRAINING){
        if(this->is_target_valid(_nd)){
            this->work_list.clear();
            auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_nd.route][_nd.node]).second;
            this->work_list.insert(this->work_list.end(), path.begin(), path.end());
            if(!this->work_list.empty()){
                this->work_list.pop_front();
            }
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
        }
        else{
            ROS_ERROR("Target out of range, plz reset");
            return false;
        }
    }
    else{
        ROS_ERROR("Not trained but want to add target");
        return false;
    }
    return true;
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
    trigger working if work_list is not empty 
    return true if the robot has entered working
    false otherwise
*/
bool Controller::trigger_working()
{
    if(this->stage_bm & MODE_TRAINING && !this->work_list.empty()){
        // check if there were jobs to do
        VertexType *next_vptr = this->work_list.front();
        // if none, enter working mode and claim.
        if(!(this->is_target_ocp(next_vptr))){
            RouteNode next_nd = this->get_affinity_vertex(this->ocp_vptr, next_vptr);
            vector<int16_t> work_args = {next_nd.route, next_nd.node};
            auto srv = this->base_driver.invoke(OPCODE_WORK_BEGIN, work_args);
            if(this->base_driver.is_invoke_valid(srv)){
                this->ocp_vptr = this->work_list.front(); // claim
                ROS_INFO("Claim the target node is occupied by the current robot");
                ROS_INFO("Enter working mode");
                // @@
                // avoid invoke working on UART repeatedly
            }
            #ifdef ROBOT_CONTROLLER_TEST
                this->mode = MODE_WORKING;
                this->stage_bm |= MODE_WORKING;
            #endif
        }
        else{
            ROS_INFO("Want to work but target is occupied, waiting");
        }
    }
}