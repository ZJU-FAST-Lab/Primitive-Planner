#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <ros/ros.h>


class CSVLogger {
private:
    std::string fileName;

    // 私有成员函数：获取当前时间并格式化为字符串
    std::string getCurrentTime() {
        std::time_t now = std::time(nullptr);
        std::tm* timeinfo = std::localtime(&now);
        std::stringstream timeStream;
        timeStream << std::put_time(timeinfo, "%Y_%m_%d_%H_%M_%S");
        return timeStream.str();
    }

public:

    CSVLogger(){}

    // 函数：在指定目录下创建一个文件夹，文件夹名为当前时间
    bool createFolderWithCurrentTime(const std::filesystem::path& baseDirectory, std::filesystem::path& newFolderPath) {
        std::string folderName = getCurrentTime();
        newFolderPath = baseDirectory / folderName;

        // 检查文件夹是否已经存在
        if (std::filesystem::exists(newFolderPath)) {
            std::cout << "Folder already exists: " << newFolderPath << std::endl;
            return true;
        }

        // 创建文件夹
        if (std::filesystem::create_directory(newFolderPath)) {
            std::cout << "Folder created successfully: " << newFolderPath << std::endl;
            return true;
        } else {
            std::cerr << "Failed to create folder: " << newFolderPath << std::endl;
            return false;
        }
    }

    // 构造函数：创建一个新的CSV文件
    void init(const std::string& path, const std::string& head, const int id = 0) {

        std::filesystem::path newFolderPath;

        if(createFolderWithCurrentTime(path, newFolderPath))
        {
            std::stringstream ss;
            ss << id;
            fileName = newFolderPath.string() + "/" + head + "_ID_" + ss.str() + "_date_" + getCurrentTime() + ".csv";
            std::ofstream file(fileName);
            if (!file.is_open()) {
                std::cerr << "Unable to create file: " << fileName << std::endl;
            }
        }
    }

    // 成员函数：在CSV文件末尾添加一行数据
    void appendData(const std::string& data) {
        std::ofstream file(fileName, std::ios::app);
        if (file.is_open()) {
            file << data << std::endl;
            file.close();
        } else {
            std::cerr << "Unable to open file: " << fileName << std::endl;
        }
    }

    // 将ros::Time和std::vector<double>转换为CSV格式的字符串
    std::string toCSV(const ros::Time& stamp, const std::vector<double>& values) {
        std::stringstream ss;
        // 将ros::Time转换为秒数并输出
        ss << stamp.sec << "." << stamp.nsec;
        // 遍历vector，输出每个值
        for (size_t i = 0; i < values.size(); ++i) {
            ss << "," << values[i]; 
        }
        // 添加换行符，如果不需要换行可以去掉这一行
        // ss << std::endl;
        return ss.str();
    }

};