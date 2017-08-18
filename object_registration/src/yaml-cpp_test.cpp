#include <ros/ros.h>
#include <ros/package.h>

#include "yaml-cpp/yaml.h"
#include <fstream>

bool test_yaml_emit(){
    try{
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "object_name";
        out << YAML::Value << "hammer";
        out << YAML::Key << "position";
        out << YAML::Value << 0.5;
        out << YAML::EndMap;
        std::cout << "Here's the output YAML:\n" << out.c_str() << std::endl; // prints "out the yaml contents


        std::ofstream fout("file_out.yaml");
        fout << out.c_str();
        return true;
    }catch(...){
        std::cout << "Failed to write out to yaml file" << std::endl;
        return false;
    }
}

bool read_yaml_file(){
    try{
        std::cout << "\nLoading yaml file" << std::endl;
        YAML::Node config = YAML::LoadFile("file_out.yaml");
        if (config["object_name"]) {
          std::cout << "object_name: " << config["object_name"].as<std::string>() << "\n";
        }
        if (config["position"]) {
          std::cout << "position: " << config["position"].as<double>() << "\n";
        }        
        return true;
    }catch(...){
        std::cout << "Failed to write out to yaml file" << std::endl;
        return false;
    }   
}


bool test_nested_maps_yaml_emit(std::string filename){
    try{
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "object_name" << YAML::Value << "hammer";
        out << YAML::Key << "box_marker_pose" << YAML::Value 
            << YAML::BeginMap                       
                 << YAML::Key << "position" << YAML::Value 
                    << YAML::BeginMap 
                        << YAML::Key << "x" << YAML::Value << 0.1  
                        << YAML::Key << "y" << YAML::Value << 0.2 
                        << YAML::Key << "z" << YAML::Value << 0.3 
                    << YAML::EndMap                               
                  << YAML::Key << "orientation" 
                    << YAML::Value << YAML::BeginMap 
                        << YAML::Key << "x" << YAML::Value << 1.1 
                        << YAML::Key << "y" << YAML::Value << 2.2
                        << YAML::Key << "z" << YAML::Value << 3.3 
                        << YAML::Key << "w" << YAML::Value << 4.4                                                         
                    << YAML::EndMap                           
            << YAML::EndMap;

        out << YAML::Key << "box_marker_size" << YAML::Value 
            << YAML::BeginMap 
                << YAML::Key << "scale_x" << YAML::Value << 1.0  
                << YAML::Key << "scale_y" << YAML::Value << 2.0 
                << YAML::Key << "scale_z" << YAML::Value << 3.0 
            << YAML::EndMap;

        out << YAML::Key << "robot_hand_tf_name" << YAML::Value << "rightPalm";
        out << YAML::Key << "grasp_relative_pose" << YAML::Value 
            << YAML::BeginMap                       
                 << YAML::Key << "position" << YAML::Value 
                    << YAML::BeginMap 
                        << YAML::Key << "x" << YAML::Value << 2.1  
                        << YAML::Key << "y" << YAML::Value << 2.2 
                        << YAML::Key << "z" << YAML::Value << 2.3 
                    << YAML::EndMap                               
                  << YAML::Key << "orientation" 
                    << YAML::Value << YAML::BeginMap 
                        << YAML::Key << "x" << YAML::Value << 2.1 
                        << YAML::Key << "y" << YAML::Value << 4.2
                        << YAML::Key << "z" << YAML::Value << 6.3 
                        << YAML::Key << "w" << YAML::Value << 8.4                                                         
                    << YAML::EndMap                           
            << YAML::EndMap;        
        
        out << YAML::EndMap;





        std::cout << "Here's the output YAML:\n" << out.c_str() << std::endl; // prints "out the yaml contents



        std::ofstream fout(filename);
        fout << out.c_str();
        return true;
    }catch(...){
        std::cout << "Failed to write out to yaml file" << std::endl;
        return false;
    }
}

bool read_nested_maps_yaml(std::string filename){
    try{
        std::cout << "\nLoading yaml file" << std::endl;
        YAML::Node baseNode = YAML::LoadFile(filename);

        YAML::Node box_marker_pose_node = baseNode["box_marker_pose"];
        YAML::Node position = box_marker_pose_node["position"];
        YAML::Node orientation = box_marker_pose_node["orientation"];        

        double pos_x = position["x"].as<double>();
        double pos_y = position["y"].as<double>();
        double pos_z = position["z"].as<double>();

        std::cout << "position x: " << pos_x << std::endl;
        std::cout << "position y: "  << pos_y << std::endl;
        std::cout << "position z: "  << pos_z << std::endl;       

        double or_x = orientation["x"].as<double>();
        double or_y = orientation["y"].as<double>();
        double or_z = orientation["z"].as<double>();
        double or_w = orientation["w"].as<double>();

        std::cout << "position x: " << pos_x << std::endl;
        std::cout << "position y: "  << pos_y << std::endl;
        std::cout << "position z: "  << pos_z << std::endl;       

        std::cout << "orientation x: " << or_x << std::endl;
        std::cout << "orientation y "  << or_y << std::endl;
        std::cout << "orientation z "  << or_z << std::endl;       
        std::cout << "orientation z "  << or_w << std::endl;       

        YAML::Node box_marker_size = baseNode["box_marker_size"];
        double size_x = box_marker_size["scale_x"].as<double>();
        double size_y = box_marker_size["scale_y"].as<double>();
        double size_z = box_marker_size["scale_z"].as<double>();

        std::cout << "size_x: " << size_x << std::endl;
        std::cout << "size_y: " << size_y << std::endl;
        std::cout << "size_z: " << size_z << std::endl;       


        return true;
    }catch(...){
        std::cout << "Failed to read yaml file" << std::endl;
        return false;
    }   
}


int main()
{
    test_yaml_emit();
    read_yaml_file();
    test_nested_maps_yaml_emit("object_grasp.yaml");    
    read_nested_maps_yaml("object_grasp.yaml");
   return 0;
}
