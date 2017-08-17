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

int main()
{
    test_yaml_emit();
    read_yaml_file();    
   return 0;
}
