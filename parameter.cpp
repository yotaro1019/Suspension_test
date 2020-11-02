#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<algorithm>
#include"parameter.h"

namespace chrono{
namespace vehicle{
Parameter::Parameter(const std::string param_fname){

        this->default_param();  //デフォルトのパラメータをよみこみ
        this->read_param(param_fname); //パラメーターファイルからの読み込み
        this->check_param(); //パラメータミスの確認

    }


//set function
std::string Parameter::Set_str_value(std::stringstream &ss){
    std::string str;
    ss >> str;
    return str;
}

int Parameter::Set_int_value(std::stringstream &ss){
    std::string str;
    int num;
    ss >> str;
    num = std::stoi(str);
    return num;  
}

double Parameter::Set_double_value(std::stringstream &ss){
    std::string str;
    double dbl;
    ss >> str;
    dbl = std::stod(str);
    return dbl;    
}

bool Parameter::Set_bool_value(std::stringstream &ss){
    std::string str;
    bool active;
    ss >> str;

    if(str == "true"){
        return true;
    }else if(str == "false"){
        return false;
    }else{
        GetLog() << "This is not bool type  \t input value = " << str << "\n";
    }
}

ChVector<> Parameter::Set_ChVector(std::stringstream &ss){
    ChVector<double> chv;
    std::string st_x,st_y,st_z;

    ss >> st_x;
    ss >> st_y;
    ss >> st_z; 
  
    chv.Set(std::stod(st_x), std::stod(st_y), std::stod(st_z));
    return chv;
}

ChQuaternion<> Parameter::Set_ChQuaternion(std::stringstream &ss){
    ChQuaternion<double> chq;
    std::string st_e1, st_e2, st_e3, st_e4;
    
    ss >> st_e1;
    ss >> st_e2;
    ss >> st_e3;
    ss >> st_e4;
    chq.Set(std::stod(st_e1), std::stod(st_e2), std::stod(st_e3), std::stod(st_e4));
    return chq;
}

std::vector<std::string> Parameter::Set_str_vec(std::stringstream &ss){
    std::vector<std::string> str_vec;
    GetLog() << str_vec.size() << "\n";
    while(!ss.eof()){
        std::string tmp;
        ss >> tmp;        
        str_vec.push_back(tmp);
    }
    str_vec.pop_back(); 
    return str_vec;
}


//use constructor
void Parameter::default_param(){
    //directory
    this->inp_dir = "inp_vehicle_models";
    this->out_dir = "output_chrono";
    
    this->veh_json = "NaN";
    this->sus_json = "NaN";
    this->tire_json = "NaN";

    this->test_axle_index = 9999999;

    this->driver_mode = "gui";

    //visualization 
    wheel_viz_type = VisualizationType::PRIMITIVES;
    tire_viz_type = VisualizationType::PRIMITIVES;
    parts_vis_type = VisualizationType::PRIMITIVES;

}

void Parameter::check_param(){
    //vehicle system (must check)
    if( this->veh_json == "NaN" && this->sus_json == "NaN"){
        this->err_msg("vehicle_json or suspension json");
    }

    if( this->tire_json == "NaN"){
        this->err_msg("tire_json");
    }

    if( this->test_axle_index == 9999999){
        this->err_msg("test_axle_index");
    }

        
  
}

void Parameter::err_msg(std::string msg){
    GetLog() << "ERROR: Please input " << msg << "\n";
}

void Parameter::read_param(std::string input_fname){
        std::ifstream inp_param_file(input_fname);
        if(inp_param_file.fail()){
            std::cout << "ERROR unable to open " << input_fname << "\n";
        }

        
        std::string str;
        
        while(getline(inp_param_file,str)){
            

            if(str[0] == '#')
                continue;

            if(str[0] == '!')
                continue;
            
            if(str.empty())
                continue;

            std::stringstream ss;
            std::string name;           

            std::replace(str.begin(), str.end(), '=', ' ');
            std::replace(str.begin(), str.end(), ',', ' ');

            ss << str;
            ss >> name;

            
            if(name == "inp_dir"){
              this->inp_dir = Set_str_value(ss);
            }

            if(name == "out_dir"){
              this->out_dir = Set_str_value(ss);
            }

            if(name == "vehicle_json"){
                this->veh_json = Set_str_value(ss);
                this->geom_json_type = "geom_veh";
            }

            if(name == "suspension_json"){
                this->sus_json = Set_str_value(ss);
                this->geom_json_type = "geom_sus";
            }

            if(name == "tire_json"){
                this->tire_json = Set_str_value(ss);
            }

            if(name == "test_axle_index"){
                this->test_axle_index = Set_int_value(ss);
            }

            if(name == "step_size"){
                this->step_size = Set_double_value(ss);
            }

            if(name == "driver_data"){
                this->driver_data = Set_str_value(ss);
                this->driver_mode = "data";
            }

        
            if(name == "parts_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->parts_vis_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->parts_vis_type= VisualizationType::PRIMITIVES;
                }else{
                    this->parts_vis_type= VisualizationType::NONE;
                }               
            }

            if(name == "wheel_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->wheel_viz_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->wheel_viz_type= VisualizationType::PRIMITIVES;
                }else{
                    this->wheel_viz_type= VisualizationType::NONE;
                }               
            }

            if(name == "tire_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->tire_viz_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->tire_viz_type= VisualizationType::PRIMITIVES;
                }else{
                    this->tire_viz_type= VisualizationType::NONE;
                }               
            }

        }

}


//output data

//bool Parameter::Get_tire_force_bool(){
//    return this->tire_force_bool;
//}
//
//std::string Parameter::Get_tire_fl_force_fname(){
//    return this->tire_fl_force_fname;
//}

}   //end namespace vehicle
}   //end namespace chrono