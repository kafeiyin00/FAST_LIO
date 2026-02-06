#include <ros/ros.h>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <sys/stat.h>

namespace fs = boost::filesystem;

// 时间戳信息结构体
struct TimestampInfo {
    uint sec;
    uint nsec;
    std::string filename;  // 不带路径的文件名
    bool has_timestamp;    // 是否成功提取时间戳
    
    TimestampInfo() : sec(0), nsec(0), filename(""), has_timestamp(false) {}
    TimestampInfo(uint s, uint ns, const std::string& name, bool valid) 
        : sec(s), nsec(ns), filename(name), has_timestamp(valid) {}
};

// 文件排序键结构体
struct FileSortKey {
    std::string filepath;
    uint64_t primary_num;    // 主要数字（秒或索引）
    uint64_t secondary_num;  // 次要数字（纳秒，如果有）
    bool is_timestamp;       // 是否为时间戳格式
    
    FileSortKey(const std::string& path) : filepath(path), primary_num(0), secondary_num(0), is_timestamp(false) {
        fs::path p(path);
        std::string stem = p.stem().string();  // 不带扩展名的文件名
        
        // 尝试解析时间戳格式: sec_nsec
        size_t underscore_pos = stem.find('_');
        if (underscore_pos != std::string::npos) {
            std::string sec_str = stem.substr(0, underscore_pos);
            std::string nsec_str = stem.substr(underscore_pos + 1);
            
            // 检查是否都是数字
            bool sec_is_digit = !sec_str.empty() && 
                std::all_of(sec_str.begin(), sec_str.end(), ::isdigit);
            bool nsec_is_digit = !nsec_str.empty() && 
                std::all_of(nsec_str.begin(), nsec_str.end(), ::isdigit);
            
            if (sec_is_digit && nsec_is_digit) {
                try {
                    primary_num = std::stoull(sec_str);
                    secondary_num = std::stoull(nsec_str);
                    is_timestamp = true;
                    return;
                } catch (const std::exception& e) {
                    // 解析失败，继续尝试简单数字
                }
            }
        }
        
        // 尝试解析为简单数字序号
        if (std::all_of(stem.begin(), stem.end(), ::isdigit) && !stem.empty()) {
            try {
                primary_num = std::stoull(stem);
                secondary_num = 0;
                is_timestamp = false;
                return;
            } catch (const std::exception& e) {
                // 解析失败，使用字符串比较
            }
        }
        
        // 如果都不是，使用文件名的哈希值（保证一致性）
        primary_num = std::hash<std::string>{}(stem);
        secondary_num = 0;
        is_timestamp = false;
    }
    
    // 比较运算符
    bool operator<(const FileSortKey& other) const {
        // 时间戳格式和数字序号分别排序，时间戳在前
        if (is_timestamp != other.is_timestamp) {
            return is_timestamp > other.is_timestamp;  // 时间戳格式优先
        }
        
        // 首先比较主要数字（秒或索引）
        if (primary_num != other.primary_num) {
            return primary_num < other.primary_num;
        }
        
        // 如果主要数字相同，比较次要数字（纳秒）
        if (secondary_num != other.secondary_num) {
            return secondary_num < other.secondary_num;
        }
        
        // 如果数字都相同，按文件名字符串比较（备用）
        return filepath < other.filepath;
    }
};

// 读取 odom 文件，返回 4x4 变换矩阵
Eigen::Matrix4d readOdomFile(const std::string& filename) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open odom file: %s", filename.c_str());
        return transform;
    }
    
    // 读取 4x4 矩阵
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file >> transform(i, j);
        }
    }
    
    file.close();
    return transform;
}

// 保存单个变换矩阵到文件
void saveTransformMatrix(const Eigen::Matrix4d& transform, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to create transform file: %s", filename.c_str());
        return;
    }
    
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file << transform(i, j);
            if (j < 3) file << " ";
        }
        file << "\n";
    }
    
    file.close();
    ROS_INFO("Saved transform to: %s", filename.c_str());
}

// 保存单个变换矩阵到文件流（带说明）
void saveTransformToStream(std::ofstream& file, 
                           const Eigen::Matrix4d& transform,
                           const std::string& from_name,
                           const std::string& to_name,
                           int idx_from = -1,
                           int idx_to = -1) {
    file << "# Transform from: " << from_name;
    if (idx_from >= 0) file << " (index: " << idx_from << ")";
    file << "\n";
    
    file << "# Transform to: " << to_name;
    if (idx_to >= 0) file << " (index: " << idx_to << ")";
    file << "\n";
    
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file << transform(i, j);
            if (j < 3) file << " ";
        }
        file << "\n";
    }
    file << "\n"; // 空行分隔
}

// 计算两个 odom 之间的相对变换: T^to_from = (T^World_to)^{-1} * T^World_from
Eigen::Matrix4d computeRelativeTransform(const Eigen::Matrix4d& odom_from, 
                                          const Eigen::Matrix4d& odom_to) {
    return odom_to.inverse() * odom_from;
}

// 获取文件夹中所有 odom 文件，按时间戳或数字序号排序
std::vector<std::string> getOdomFiles(const std::string& folder_path) {
    std::vector<std::string> odom_files;
    
    if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
        ROS_ERROR("Folder does not exist: %s", folder_path.c_str());
        return odom_files;
    }
    
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".odom") {
            odom_files.push_back(entry.path().string());
        }
    }
    
    // ✅ 使用智能排序（同时处理时间戳和数字序号）
    std::vector<FileSortKey> sort_keys;
    for (const auto& file : odom_files) {
        sort_keys.emplace_back(file);
    }
    
    std::sort(sort_keys.begin(), sort_keys.end());
    
    // 提取排序后的文件路径
    odom_files.clear();
    for (const auto& key : sort_keys) {
        odom_files.push_back(key.filepath);
    }
    
    ROS_INFO("Found %zu odom files in folder: %s", odom_files.size(), folder_path.c_str());
    
    // 打印前几个和后几个文件名用于验证
    /*
    if (!odom_files.empty()) {
        size_t show_count = std::min(size_t(5), odom_files.size());
        ROS_INFO("First %zu files:", show_count);
        for (size_t i = 0; i < show_count; i++) {
            fs::path p(odom_files[i]);
            ROS_INFO("  [%zu] %s", i, p.filename().string().c_str());
        }
        
        if (odom_files.size() > show_count) {
            ROS_INFO("Last %zu files:", show_count);
            for (size_t i = odom_files.size() - show_count; i < odom_files.size(); i++) {
                fs::path p(odom_files[i]);
                ROS_INFO("  [%zu] %s", i, p.filename().string().c_str());
            }
        }
    }
    */
    
    return odom_files;
}

// 从文件名提取时间戳（假设格式为 sec_nsec.odom）
// 如果无法提取时间戳，至少返回文件名
TimestampInfo extractTimestamp(const std::string& filepath) {
    fs::path p(filepath);
    std::string filename = p.filename().string();  // 获取不带路径的文件名
    std::string stem = p.stem().string();          // 获取不带扩展名的文件名
    
    // 尝试从文件名提取时间戳（格式: sec_nsec.odom）
    size_t underscore_pos = stem.find('_');
    if (underscore_pos != std::string::npos) {
        try {
            std::string sec_str = stem.substr(0, underscore_pos);
            std::string nsec_str = stem.substr(underscore_pos + 1);
            
            // 检查是否都是数字
            bool sec_is_digit = !sec_str.empty() && 
                std::all_of(sec_str.begin(), sec_str.end(), ::isdigit);
            bool nsec_is_digit = !nsec_str.empty() && 
                std::all_of(nsec_str.begin(), nsec_str.end(), ::isdigit);
            
            if (sec_is_digit && nsec_is_digit) {
                uint sec = std::stoul(sec_str);
                uint nsec = std::stoul(nsec_str);
                return TimestampInfo(sec, nsec, filename, true);
            }
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse timestamp from filename '%s': %s", 
                     filename.c_str(), e.what());
        }
    }
    
    // 如果无法提取时间戳，返回文件名作为标识
    //ROS_WARN("Could not extract timestamp from '%s', using filename as identifier", 
    //         filename.c_str());
    return TimestampInfo(0, 0, filename, false);
}

// 模式1: 任意两两计算（所有组合）
void computeAllPairs(const std::vector<std::string>& odom_files, 
                     const std::string& output_folder) {
    ROS_INFO("Computing transforms for all pairs (bidirectional)...");
    
    // 创建输出文件
    std::string output_filename = output_folder + "/all_transforms.txt";
    std::ofstream output_file(output_filename);
    
    if (!output_file.is_open()) {
        ROS_ERROR("Failed to create output file: %s", output_filename.c_str());
        return;
    }
    
    // 写入文件头
    output_file << "# All Pairwise Transforms (Bidirectional)\n";
    output_file << "# Generated by odom_transformer\n";
    output_file << "# Total odom files: " << odom_files.size() << "\n";
    output_file << "# Total transform pairs: " << (odom_files.size() * (odom_files.size() - 1) / 2) << " (bidirectional)\n";
    output_file << "# Format: 4x4 transformation matrix (from -> to and to -> from)\n";
    output_file << "########################################\n\n";
    
    int pair_count = 0;
    for (size_t i = 0; i < odom_files.size(); i++) {
        for (size_t j = i + 1; j < odom_files.size(); j++) {
            Eigen::Matrix4d odom_i = readOdomFile(odom_files[i]);
            Eigen::Matrix4d odom_j = readOdomFile(odom_files[j]);
            
            // 计算 i -> j 的变换
            Eigen::Matrix4d transform_i_to_j = computeRelativeTransform(odom_i, odom_j);
            // 计算 j -> i 的变换（逆变换）
            Eigen::Matrix4d transform_j_to_i = transform_i_to_j.inverse();
            
            auto ts_i = extractTimestamp(odom_files[i]);
            auto ts_j = extractTimestamp(odom_files[j]);
            
            // 生成标识名称（使用时间戳或文件名）
            char from_name[512], to_name[512];
            if (ts_i.has_timestamp) {
                sprintf(from_name, "%u_%u", ts_i.sec, ts_i.nsec);
            } else {
                snprintf(from_name, sizeof(from_name), "%s", ts_i.filename.c_str());
            }
            
            if (ts_j.has_timestamp) {
                sprintf(to_name, "%u_%u", ts_j.sec, ts_j.nsec);
            } else {
                snprintf(to_name, sizeof(to_name), "%s", ts_j.filename.c_str());
            }
            
            // 输出 i -> j
            output_file << "## Pair " << ++pair_count << " (Forward: " << i << " -> " << j << ")\n";
            saveTransformToStream(output_file, transform_i_to_j, from_name, to_name, i, j);
            
            // 输出 j -> i
            output_file << "## Pair " << pair_count << " (Reverse: " << j << " -> " << i << ")\n";
            saveTransformToStream(output_file, transform_j_to_i, to_name, from_name, j, i);
            
            output_file << "--------------------\n"; // 分隔不同的配对组
        }
    }
    
    output_file.close();
    ROS_INFO("Saved %d bidirectional transform pairs to: %s", pair_count, output_filename.c_str());
}

// 模式2: 按前后顺序两两计算
void computeSequentialPairs(const std::vector<std::string>& odom_files, 
                            const std::string& output_folder) {
    ROS_INFO("Computing transforms for sequential pairs (bidirectional)...");
    
    // 创建输出文件
    std::string output_filename = output_folder + "/sequential_transforms.txt";
    std::ofstream output_file(output_filename);
    
    if (!output_file.is_open()) {
        ROS_ERROR("Failed to create output file: %s", output_filename.c_str());
        return;
    }
    
    // 写入文件头
    output_file << "# Sequential Pairwise Transforms (Bidirectional)\n";
    output_file << "# Generated by odom_transformer\n";
    output_file << "# Total odom files: " << odom_files.size() << "\n";
    output_file << "# Total sequential pairs: " << (odom_files.size() - 1)  << " (bidirectional)\n";
    output_file << "# Format: 4x4 transformation matrix (from i -> to i+1 and i+1 -> i)\n";
    output_file << "########################################\n\n";
    
    for (size_t i = 0; i < odom_files.size() - 1; i++) {
        Eigen::Matrix4d odom_i = readOdomFile(odom_files[i]);
        Eigen::Matrix4d odom_i1 = readOdomFile(odom_files[i + 1]);
        
        // 计算 i -> i+1 的变换
        Eigen::Matrix4d transform_forward = computeRelativeTransform(odom_i, odom_i1);
        // 计算 i+1 -> i 的变换（逆变换）
        Eigen::Matrix4d transform_reverse = transform_forward.inverse();
        
        auto ts_i = extractTimestamp(odom_files[i]);
        auto ts_i1 = extractTimestamp(odom_files[i + 1]);
        
        // 生成标识名称
        char from_name[512], to_name[512];
        if (ts_i.has_timestamp) {
            sprintf(from_name, "%u_%u", ts_i.sec, ts_i.nsec);
        } else {
            snprintf(from_name, sizeof(from_name), "%s", ts_i.filename.c_str());
        }
        
        if (ts_i1.has_timestamp) {
            sprintf(to_name, "%u_%u", ts_i1.sec, ts_i1.nsec);
        } else {
            snprintf(to_name, sizeof(to_name), "%s", ts_i1.filename.c_str());
        }
        
        // 输出前向变换 i -> i+1
        output_file << "## Sequential Pair " << (i + 1) << " (Forward: " << i << " -> " << (i+1) << ")\n";
        saveTransformToStream(output_file, transform_forward, from_name, to_name, i, i + 1);
        
        // 输出反向变换 i+1 -> i
        output_file << "## Sequential Pair " << (i + 1) << " (Reverse: " << (i+1) << " -> " << i << ")\n";
        saveTransformToStream(output_file, transform_reverse, to_name, from_name, i + 1, i);
        
        output_file << "--------------------\n"; // 分隔不同的配对组
    }
    
    output_file.close();
    ROS_INFO("Saved %zu bidirectional sequential transforms to: %s", (odom_files.size() - 1) , output_filename.c_str());
}

// 模式3: 根据索引数组计算
// 索引数组为二维数组，每个子数组包含两个元素：[[0,1], [2,3], [4,5]]
void computeIndexedPairs(const std::vector<std::string>& odom_files, 
                         const std::vector<std::vector<int>>& index_pairs,
                         const std::string& output_folder) {
    ROS_INFO("Computing transforms for indexed pairs (bidirectional)...");
    
    // 创建输出文件
    std::string output_filename = output_folder + "/indexed_transforms.txt";
    std::ofstream output_file(output_filename);
    
    if (!output_file.is_open()) {
        ROS_ERROR("Failed to create output file: %s", output_filename.c_str());
        return;
    }
    
    // 写入文件头
    output_file << "# Indexed Pairwise Transforms (Bidirectional)\n";
    output_file << "# Generated by odom_transformer\n";
    output_file << "# Total odom files: " << odom_files.size() << "\n";
    output_file << "# Total indexed pairs: " << index_pairs.size() << " (bidirectional)\n";
    output_file << "# Format: 4x4 transformation matrix (from specified index pairs, both directions)\n";
    output_file << "########################################\n\n";
    
    int valid_pairs = 0;
    for (size_t i = 0; i < index_pairs.size(); i++) {
        if (index_pairs[i].size() != 2) {
            ROS_WARN("Index pair %zu does not have exactly 2 elements, skipping...", i);
            output_file << "## Pair " << (i + 1) << " - SKIPPED (invalid format)\n\n";
            continue;
        }
        
        int idx1 = index_pairs[i][0];
        int idx2 = index_pairs[i][1];
        
        // 验证索引有效性
        if (idx1 < 0 || idx1 >= (int)odom_files.size() || 
            idx2 < 0 || idx2 >= (int)odom_files.size()) {
            ROS_WARN("Invalid index pair: (%d, %d), skipping...", idx1, idx2);
            output_file << "## Pair " << (i + 1) << " - SKIPPED (invalid indices: " 
                       << idx1 << ", " << idx2 << ")\n\n";
            continue;
        }
        
        // 读取两个 odom 文件
        Eigen::Matrix4d odom_1 = readOdomFile(odom_files[idx1]);
        Eigen::Matrix4d odom_2 = readOdomFile(odom_files[idx2]);
        
        // 计算 idx1 -> idx2 的变换
        Eigen::Matrix4d transform_forward = computeRelativeTransform(odom_1, odom_2);
        // 计算 idx2 -> idx1 的变换（逆变换）
        Eigen::Matrix4d transform_reverse = transform_forward.inverse();
        
        // 提取时间戳
        auto ts_1 = extractTimestamp(odom_files[idx1]);
        auto ts_2 = extractTimestamp(odom_files[idx2]);
        
        // 生成标识名称
        char from_name[512], to_name[512];
        if (ts_1.has_timestamp) {
            sprintf(from_name, "%u_%u", ts_1.sec, ts_1.nsec);
        } else {
            snprintf(from_name, sizeof(from_name), "%s", ts_1.filename.c_str());
        }
        
        if (ts_2.has_timestamp) {
            sprintf(to_name, "%u_%u", ts_2.sec, ts_2.nsec);
        } else {
            snprintf(to_name, sizeof(to_name), "%s", ts_2.filename.c_str());
        }
        
        // 输出前向变换 idx1 -> idx2
        output_file << "## Indexed Pair " << (i + 1) << " (Forward: " << idx1 << " -> " << idx2 << ")\n";
        saveTransformToStream(output_file, transform_forward, from_name, to_name, idx1, idx2);
        
        // 输出反向变换 idx2 -> idx1
        output_file << "## Indexed Pair " << (i + 1) << " (Reverse: " << idx2 << " -> " << idx1 << ")\n";
        saveTransformToStream(output_file, transform_reverse, to_name, from_name, idx2, idx1);
        
        output_file << "--------------------\n"; // 分隔不同的配对组
        
        valid_pairs++;
        ROS_INFO("Computed bidirectional transform for pair %zu: index %d <-> index %d", 
                 i + 1, idx1, idx2);
    }
    
    output_file.close();
    ROS_INFO("Saved %d bidirectional indexed transforms to: %s", valid_pairs , output_filename.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_transformer");
    ros::NodeHandle nh("~");
    
    // 读取参数
    std::string odom_folder, output_folder;
    std::string mode; // "all", "sequential", "indexed"
    std::vector<std::vector<int>> index_pairs;
    
    nh.param<std::string>("odom_folder", odom_folder, "");
    nh.param<std::string>("output_folder", output_folder, "");
    nh.param<std::string>("mode", mode, "sequential");
    
    if (odom_folder.empty() || output_folder.empty()) {
        ROS_ERROR("Please specify odom_folder and output_folder parameters!");
        return -1;
    }
    
    // 检查并创建输出文件夹
    struct stat info;
    if (stat(output_folder.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        ROS_WARN("Output folder does not exist. Creating: %s", output_folder.c_str());
        mkdir(output_folder.c_str(), 0777);
    }
    
    // 获取所有 odom 文件
    std::vector<std::string> odom_files = getOdomFiles(odom_folder);
    
    if (odom_files.empty()) {
        ROS_ERROR("No odom files found in folder: %s", odom_folder.c_str());
        return -1;
    }
    
    ROS_INFO("Processing mode: %s", mode.c_str());
    
    // 根据模式执行不同的计算
    if (mode == "all") {
        computeAllPairs(odom_files, output_folder);
    } else if (mode == "sequential") {
        computeSequentialPairs(odom_files, output_folder);
    } else if (mode == "indexed") {
        // 读取二维索引数组
        XmlRpc::XmlRpcValue indices_param;
        if (nh.getParam("indices", indices_param)) {
            if (indices_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < indices_param.size(); i++) {
                    if (indices_param[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                        std::vector<int> pair;
                        for (int j = 0; j < indices_param[i].size(); j++) {
                            pair.push_back(static_cast<int>(indices_param[i][j]));
                        }
                        index_pairs.push_back(pair);
                    } else {
                        ROS_WARN("Element %d in indices is not an array, skipping...", i);
                    }
                }
            }
        }
        
        if (index_pairs.empty()) {
            ROS_ERROR("Indexed mode requires 'indices' parameter with valid pairs!");
            return -1;
        }
        
        ROS_INFO("Loaded %zu index pairs", index_pairs.size());
        computeIndexedPairs(odom_files, index_pairs, output_folder);
    } else {
        ROS_ERROR("Unknown mode: %s. Use 'all', 'sequential', or 'indexed'", mode.c_str());
        return -1;
    }
    
    ROS_INFO("Transform computation completed!");
    
    return 0;
}