#pragma once
#include <lego/Utils/Math.hpp>
#include <lego/Utils/ErrorHandling.hpp>

namespace lego_manipulation
{
namespace io
{

typedef std::map<std::string, std::vector<double>> ConfigMap;

typedef enum : int {
    Robot,
    Obstacle,
    Size
} CapType;

std::vector<math::Capsule> LoadCapsulesFromFile(const std::string& fname, const CapType& cap_type);
std::vector<math::BoundingBox> LoadBoundingBoxFromFile(const std::string& fname);

template <typename T>
std::string vtos(const std::vector<T>& vec)
{
    std::ostringstream ss;
    for (const auto& val:vec)
    {
        ss << val << " ";
    }
    return ss.str();
}

ConfigMap ParseConfigFile(const std::string fname);

template <typename T>
void ParseVariable(const std::string& key, T& var, ConfigMap& dict)
{
    try{
        var = (dict[key]).at(0);
    }
    catch(...)
    {
        std::cerr << "Failed to load " << key << ", using default value " << var << "\n";
    }
}

template <typename T>
void ParseVariable(const std::string& key, std::vector<T>& var, ConfigMap& dict)
{
    try{
        std::vector<T> vec;
        for (auto i = dict[key].begin(); i != dict[key].end(); ++i){
            vec.push_back((T) *i);
        }
        var.assign(vec.begin(), vec.end());
    }
    catch(...)
    {
        std::cerr << "Failed to load " << key << ", using default value " << vtos(var) << "\n";
    }
}

Eigen::MatrixXd LoadMatFromFile(const std::string fname);
void SaveMatToFile(const Eigen::MatrixXd& mat, const std::string& fname);

}

}