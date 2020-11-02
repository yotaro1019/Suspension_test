#ifndef _read_init_data_
#define _read_init_data_
#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/ChSubsysDefs.h"


namespace chrono{
namespace vehicle{

class Parameter{
private:
    //constructor
    void default_param();
    void check_param();
    void err_msg(std::string msg);
    void read_param(std::string input_fname);


    //dir_path
    std::string inp_dir, out_dir;

    //JSON fname
    std::string veh_json, sus_json, tire_json;
    std::string geom_json_type;     //geom_veh or geom_sus

    
    int test_axle_index;    //index of test axle
    double step_size;

    std::string driver_mode, driver_data;

    VisualizationType wheel_viz_type, tire_viz_type, parts_vis_type;
   
  


public:
    Parameter(const std::string param_fname);

    //set function
    std::string Set_str_value(std::stringstream &ss);
    int Set_int_value(std::stringstream &ss);
    double Set_double_value(std::stringstream &ss);
    bool Set_bool_value(std::stringstream &ss);
    ChVector<> Set_ChVector(std::stringstream &ss);
    ChQuaternion<> Set_ChQuaternion(std::stringstream &ss);
    std::vector<std::string> Set_str_vec(std::stringstream &ss);

    //dir
    std::string Get_inp_dir(){ return inp_dir; }
    std::string Get_out_dir(){ return out_dir; }
    
   
    //call function 
    std::string Get_veh_json(){ return this->veh_json; }
    std::string Get_sus_json(){ return this->sus_json; }
    std::string Get_tire_json(){ return this->tire_json; }
    std::string Get_geom_type(){ return this->geom_json_type; }

    int Get_test_axle_index(){ return this->test_axle_index; }
    double Get_step_size(){ return this->step_size; }
    std::string Get_driver_mode(){ return this->driver_mode; }
    std::string Get_driver_data(){ return this->driver_data; }

    //visualization  
    VisualizationType Get_wheel_viz_type(){ return this->wheel_viz_type; }
    VisualizationType Get_tire_viz_type(){ return this->tire_viz_type; }
    VisualizationType Get_parts_viz_type(){ return this->parts_vis_type; }

};

}
}
#endif