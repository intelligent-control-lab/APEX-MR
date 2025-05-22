/*
***********************************************************************************************************************************************************************
Copyright notice for IP Docket # 2022-013, The CFS Library in C++ for Robot Arm Trajectory Planning 2020 Carnegie Mellon University. 
All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written permission.

Rui Chen      ruic3@andrew.cmu.edu
Ruixuan Liu   ruixuanl@andrew.cmu.edu
Weiye Zhao    weiyezha@andrew.cmu.edu
Changliu Liu  cliu6@andrew.cmu.edu

A023793. Sponsor is provided a non-exclusive, world-wide license to the Licensed Technology in the field of robotic painting and arcwelding.

This notice must appear in all copies of this file and its derivatives.
***********************************************************************************************************************************************************************
*/
#include <lego/Utils/FileIO.hpp>

namespace lego_manipulation
{
namespace io
{

std::vector<math::Capsule> LoadCapsulesFromFile(const std::string& fname, const CapType& cap_type)
{
    try
    {
        std::string fname_{fname};
        /* code */
        if(fname_.compare("") == 0 && cap_type == CapType::Robot)
        {
            std::cout << "Using default capsule file for [robot]\n";
            fname_ = "config/geometry_wrappers/robot_capsules.txt";
        }
        else if(fname_.compare("") == 0 && cap_type == CapType::Obstacle)
        {
            std::cout << "Using default capsule file for [obstacle]\n";
            fname_ = "config/geometry_wrappers/obs_capsules.txt";
        }

        if(cap_type == CapType::Robot){
            std::cout << "Load Robot Capsules from " << fname_ << std::endl;
        }
        else{
            std::cout << "Load Obstacles Capsules from " << fname_ << std::endl;
        }
        
        std::vector<math::Capsule> tmp_capsule;

        std::ifstream file(fname_);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << ERR_HEADER << fname_ << " not found.";
            throw std::runtime_error(ss.str());
        }
        std::string data;
        std::vector<double> row;

        while(std::getline(file, data))
        {
            row.clear();
            std::string word = ""; 
            for (auto x : data){
                if (x == ',') 
                {    
                    row.push_back(std::stod(word));
                    word = "";
                } 
                else
                { 
                    word = word + x; 
                } 
            }
            row.push_back(std::stod(word)); 
            if(cap_type == CapType::Robot){
                double r = row.at(0);
                Eigen::MatrixXd p(3, 2);
                p << row.at(1), row.at(4), 
                     row.at(2), row.at(5), 
                     row.at(3), row.at(6);
                math::Capsule cap;
                cap.p = p;
                cap.r = r;
                tmp_capsule.push_back(cap); 
            }
            else{
                if(row.at(0) == 1){
                    double r = row.at(1);
                    Eigen::MatrixXd p(3, 2);
                    p << row.at(2), row.at(5), 
                        row.at(3), row.at(6), 
                        row.at(4), row.at(7);
                    Eigen::Vector3d vel;
                    vel << row.at(28), row.at(29), row.at(30);
                    math::Capsule cap;
                    cap.p = p;
                    cap.r = r;
                    cap.vel = vel;
                    tmp_capsule.push_back(cap); 
                }
            }
        }
        file.close();
        return tmp_capsule;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}


std::vector<math::BoundingBox> LoadBoundingBoxFromFile(const std::string& fname)
{
    try
    {
        std::string fname_{fname};
        /* code */
        if(fname_.compare("") == 0)
        {
            std::cout << "Using default capsule file for [obstacle bounding box]\n";
            fname_ = "config/geometry_wrappers/obs_capsules.txt";
        }
        std::cout << "Load Obstacles Bounding Boxes from " << fname_ << std::endl;
        
        std::vector<math::BoundingBox> tmp_bounding_boxes;

        std::ifstream file(fname_);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << ERR_HEADER << fname_ << " not found.";
            throw std::runtime_error(ss.str());
        }
        std::string data;
        std::vector<double> row;

        while(std::getline(file, data))
        {
            row.clear();
            std::string word = ""; 
            for (auto x : data){
                if (x == ',') 
                {    
                    row.push_back(std::stod(word));
                    word = "";
                } 
                else
                { 
                    word = word + x; 
                } 
            }
            row.push_back(std::stod(word)); 
            if(row.at(0) == 0){
                math::BoundingBox box;
                Eigen::Vector3d p(3);
                p << row.at(1), row.at(2), row.at(3);
                box.p1_1 = p;
                p << row.at(4), row.at(5), row.at(6);
                box.p1_2 = p;
                p << row.at(7), row.at(8), row.at(9);
                box.n1 = p;

                p << row.at(10), row.at(11), row.at(12);
                box.p2_1 = p;
                p << row.at(13), row.at(14), row.at(15);
                box.p2_2 = p;
                p << row.at(16), row.at(17), row.at(18);
                box.n2 = p;

                p << row.at(19), row.at(20), row.at(21);
                box.p3_1 = p;
                p << row.at(22), row.at(23), row.at(24);
                box.p3_2 = p;
                p << row.at(25), row.at(26), row.at(27);
                box.n3 = p;

                Eigen::Vector3d vel;
                vel << row.at(28), row.at(29), row.at(30);
                box.vel = vel;
                
                tmp_bounding_boxes.push_back(box); 
            }
        }
        file.close();
        return tmp_bounding_boxes;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

ConfigMap ParseConfigFile(const std::string fname)
{
    try
    {
        std::ifstream file(fname);

        if (!file.is_open())
        {
            throw std::runtime_error("Cannot open " + fname);
        }

        std::cout << "Loading config from " << fname << std::endl;

        std::string data;
        std::string key;
        std::vector<double> vec;
        std::map<std::string, std::vector<double>> dicts;
        while(std::getline(file, data)) {
            
            // skip empty line and comments
            if (data.length() == 0 || data.at(0) == '#')
            {
                continue;
            }

            key.clear();
            vec.clear();
            /*
             * make sure each txt line has no ' ' in the end
             * make sure there is '\n' at end of txt file 
             */

            bool first_element{true};
            std::string word = ""; 
            for (auto x : data){
                if (x == ' ' || x == '\n')
                {
                    if(first_element){
                        // get key
                        key.assign(word);
                        word = ""; // empty word
                        first_element = false;
                    }
                    else{
                        // get vec
                        vec.push_back(std::stod(word)); // push back next element
                        word = ""; // empty word
                    }
                } 
                else
                { 
                    word = word + x; 
                } 
            }
            vec.push_back(std::stod(word)); 
            dicts.insert(std::pair<std::string, std::vector<double> >(key, vec));
        }

        file.close();

        // set the input value
        return dicts;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

Eigen::MatrixXd LoadMatFromFile(const std::string fname)
{
    try
    {
        Eigen::MatrixXd mat(0, 0);
        std::ifstream file(fname);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "Cannot open file " << fname;
            throw std::runtime_error(ss.str());
        }
        std::string line;
        std::vector<double> row;
        while (std::getline(file, line))
        {
            row.clear();
            std::string word = ""; 
            for (auto x : line){
                if (x == '\t' || x == ' ' || x == ',') 
                {    
                    row.push_back(std::stod(word));
                    word = "";
                } 
                else
                { 
                    word = word + x; 
                } 
            }
            row.push_back(std::stod(word));
            mat = math::EigenVcat(mat, math::ToEigen(row).transpose());
        }

        file.close();

        std::cout << "Loaded mat of shape [" << mat.rows()
                    << ", " << mat.cols() << "] from [" << fname << "]\n";

        return mat;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void SaveMatToFile(const Eigen::MatrixXd& mat, const std::string& fname)
{
    try
    {
        std::ofstream file(fname);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "Cannot write to file " << fname;
            throw std::runtime_error(ss.str());
        }
        Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n", "", "", "", "");
        file << mat.format(fmt);
        file.close();

        std::cout << "Wrote mat of shape [" << mat.rows()
                    << ", " << mat.cols() << "] to [" << fname << "]\n";
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}
}