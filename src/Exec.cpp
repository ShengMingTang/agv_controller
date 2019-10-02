#include <ros/ros.h>
#include "pybot"
#include "Proc.h"

template<typename JoyInter>
void Controller::exec(Ptr<JoyInter> _ptr)
{
    if(_ptr && _ptr->ptr){
        auto joy_sig = _ptr->ptr;
        this->op = this->decode_opcode(joy_sig);
        // no need to expand
        switch(this->op){
            // return statements
            case Opcode::OPCODE_POWEROFF:
                this->log();
                return;
            case Opcode::OPCODE_IDLE:
                ROS_WARN("Direct forced switch to Idle mode");
                #if CONTROLLER_TEST
                    this->mode = Mode::MODE_IDLE;
                #endif
                break;
            // privileged instructions above
            case Opcode::OPCODE_HOMEING:
                srv = this->base_driver.invoke((char)Opcode::OPCODE_HOMEING, vector<int16_t>());
                #if CONTROLLER_TEST
                    this->mode = Mode::MODE_HOMING;
                #endif
                this->clear();
                this->tasks.push( Ptr<Homing>(new Homing()) );
                ROS_INFO("Switch to Homing");
                break;
            case Opcode::OPCODE_TRAIN_BEGIN:
                srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_BEGIN, vector<int16_t>());
                #if CONTROLLER_TEST
                    this->mode = Mode::MODE_TRAINING;
                    node_ct = 0;
                    this->training_route = route_ct++;
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->training_route = srv.response.feedback[0];
                    }
                #endif
                this->tasks.push( Ptr<Training>(new Training()) );
                ROS_INFO("Switch to Training");
                break;
            case Opcode::OPCODE_WORK_BEGIN:
                srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, vector<int16_t>());
                #if CONTROLLER_TEST
                    this->mode = Mode::MODE_WORKING;
                #endif
                break;
            case Opcode::OPCODE_DRIVE:
                //set v, w
                this->tasks.push( Ptr<Drive>(new Drive(this->pose_tracer.get_v(), this->pose_tracer.get_w())) );
            
            default:
                break;
        }
        // no need to repush
    }
}

