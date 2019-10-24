#include "Controller.h"
void Controller::loopOnce()
{
    RobotInvoke srv;
    this->op_ptr = this->get_joy_signal();
    this->op = this->decode_opcode(op_ptr);
    // routine
    this->check_safety();
    // wifi implementation !
    if(!this->wifi.empty()){
        // processing wifi
    }
    else{
        // decode privileged instructions
        switch(this->op){
            // return statements
            case Opcode::OPCODE_POWEROFF:
                srv = this->base_driver.invoke((char)Opcode::OPCODE_POWEROFF, vector<int16_t>());
                if(this->base_driver.is_invoke_valid(srv)){
                    this->log();
                    this->is_ok = false;
                    ROS_INFO("Power off!");
                }
                else{
                    ROS_ERROR("<Poweroff Srv-Err>");
                }
                break;
            default:
                switch(this->mode){
                    case Mode::MODE_IDLE:
                        this->idle();
                        break;
                    case Mode::MODE_HOMING:
                        this->homing();
                        break;
                    case Mode::MODE_TRAINING:
                        if(this->is_calibed)
                            this->training();
                        else{
                            ROS_WARN("Not Homed but want to enter Trainng mode");
                            ROS_WARN("Direct forced switch to Idle mode");
                            this->mode = Mode::MODE_IDLE;
                        }
                        break;
                    case Mode::MODE_WORKING:
                        if(this->is_trained)
                            this->working();
                        else{
                            ROS_WARN("Not Trained but want to enter Working mode");
                            ROS_WARN("Direct forced switch to Idle mode");
                            this->mode = Mode::MODE_IDLE;
                        }
                        break;
                    default:
                        ROS_WARN("Undefined mode");
                        break;
                }
                break;
        }
    }
    this->op_ptr = nullptr;
    this->op = Opcode::OPCODE_NONE;
    #if AGV_CONTROLLER_VERBOSE
        this->monitor_display();
    #endif
}
void Controller::idle()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_HOMEING:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_HOMEING, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
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
            vector<int16_t> train_args = {this->training_route};
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_BEGIN, train_args);
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_TRAINING;
                node_ct = 0;
                this->training_route = route_ct++;
                ROS_INFO("Start Training @ R%d", this->training_route);
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Start Training @ R%d", this->training_route);
                }
                else{
                    ROS_ERROR("<Traing begin Srv-Err>");
                }
            #endif
            break;
        }
        case Opcode::OPCODE_WORK_BEGIN:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, vector<int16_t>(this->target_rn.route, this->target_rn.node));
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_WORKING;
                ROS_INFO("Start Working !!");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Start Working !! Not implemented");
                }
                else{
                    ROS_ERROR("<Working begin Srv-Err>");
                }
            #endif
            break;
        default:
            // who to fill work_list
            if(!this->work_list.empty()){
                RouteNode next_rn = *(this->work_list.begin());
                if(this->node_ocp.route != next_rn.route || this->node_ocp.node != next_rn.node){
                    bool is_ocp = this->wifi.node_ocp(next_rn); // ask if the next node is occupied
                    if(is_ocp){ // true then the node is occupied
                        // wait
                        ROS_INFO("Node occupied, waiting");
                    }
                    else{ // else claim that we have occipied this node
                        this->node_ocp = this->work_list.front(); // claim
                        vector<int16_t> work_args{this->node_ocp.route, this->node_ocp.node};
                        srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, work_args);
                        if(this->base_driver.is_invoke_valid(srv)){
                            ROS_INFO("Enter Working mode");
                        }
                        else{
                            ROS_ERROR("<Working begin Srv-Err>");
                        }
                    }
                }
            }
            else{
                ROS_WARN("In Idle mode, %c is not allowed", (char)this->op);
            }
            break;
    }
    this->drive();
}
void Controller::homing()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_ORIGIN:
            if(!this->is_calib_begin){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_ORIGIN, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
                    this->pose_tracer.clear();
                    this->is_origin_set = true;
                    ROS_INFO("Origin set");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){\
                        this->pose_tracer.clear();
                        ROS_INFO("Origin set");
                        this->is_origin_set = true;
                    }
                    else{
                        ROS_ERROR("<Origin set Srv-Err>");
                    }
                #endif
            }
            break;
        case Opcode::OPCODE_CALIB_BEGIN:
            if(this->is_origin_set && !this->is_calib_begin){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_BEGIN, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
                    this->is_calib_begin = true;
                    ROS_INFO("Calib begin");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->is_calib_begin = true;
                        ROS_INFO("Calibration begin");
                    }
                    else{
                        ROS_ERROR("<Calib begin Srv-Err>");
                    }
                #endif
            }
            else{
                if(!this->is_origin_set)
                    ROS_ERROR("[Calib begin Err], Set Origin first plz");
                else if(this->is_calib_begin)
                    ROS_ERROR("[Calib begin Err], calib already started");
            }
            break;
        case Opcode::OPCODE_CALIB_FINISH:
            if(this->is_calib_begin){
                this->is_calibed = true;
                this->is_calib_begin = false;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_FINISH, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
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
    this->drive();
}
void Controller::training()
{
    RobotInvoke srv;
    geometry_msgs::Quaternion pos;
    // node
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_SETNODE:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                // modify response
                srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
                srv.response.error_code = (int16_t)Errcode::ERRCODE_OK; // ok
                srv.response.feedback = vector<int16_t>(1, node_ct++);
            #endif
                if(this->base_driver.is_invoke_valid(srv)){
                    // graph routine
                    RouteNode nd;
                    nd.route = this->training_route, nd.node = srv.response.feedback[0];
                    pos = this->pose_tracer.get_coor();
                    this->graph.add_node(nd, this->pose_tracer.get_dist());
                    this->pose_tracer.reset_path();
                    ROS_INFO("Node R%d, N%d @ (%f,%f)", nd.route, nd.node, pos.x, pos.y);
                }
                else{
                    ROS_ERROR("<SetNode Srv-Err>");
                }
            break;
        case Opcode::OPCODE_TRAIN_FINISH:
            this->is_trained = true;
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_FINISH, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_IDLE;
                this->training_route++;
                ROS_INFO("Training finished");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->training_route++;
                    ROS_INFO("Training finished, training route inc");
                }
                else{
                    ROS_ERROR("<Traing finished Srv-Err>");
                }
            #endif
            break;
        default:
            ROS_INFO("In Training mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive();
}
/* just simply polling tracking status */
void Controller::working()
{
    if(this->tracking_status == Tracking_status::TRACKING_STATUS_ARRIVAL){
        auto srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_FINISH, vector<int16_t>());
        if(this->base_driver.is_invoke_valid(srv)){
            this->work_list.pop_front();
            // claim that we have given up that occupied node
            this->node_ocp.route = this->node_ocp.node = -1;
            ROS_INFO("Work finished");
        }
        else{
            ROS_ERROR("<Working finish Srv-Err>");
        }
    }
}