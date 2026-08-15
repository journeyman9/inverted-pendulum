#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include "kalman.h"

struct rowData {
    float timestamp;
    float x;
    float xdot;
    float theta;
    float thetadot;
    float u;
};

struct exportData {
    float timestamp;
    float x;
    float xdot;
    float theta;
    float thetadot;
    float u;
    float x_hat_0;
    float x_hat_1;
    float x_hat_2;
    float x_hat_3;
    float trace_P;
    float trace_Kf;
    
    std::string to_csv_line() const {
        std::stringstream ss;
        ss << timestamp << "," << x << "," << xdot << "," << theta << "," << thetadot << "," << u << ","
           << x_hat_0 << "," << x_hat_1 << "," << x_hat_2 << "," << x_hat_3 << "," << trace_P << "," << trace_Kf;
        return ss.str();
    }
};

void writeCSV(const std::string& filepath, const std::vector<exportData>& dataset) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filepath << std::endl;
        return;
    }
    
    file << "timestamp,x,xdot,theta,thetadot,u,x_hat_0,x_hat_1,x_hat_2,x_hat_3,trace_P,trace_Kf\n";
    
    for (const auto& row: dataset) {
        file << row.to_csv_line() << "\n";
    }
    file.close();
    std::cout << "Successfully wrote " << dataset.size() << " rows to " << filepath << std::endl;
}


int main() {
    std::string input_file = "./logged_data/normal_operation/data-timestamps-200-001.csv";
    std::ifstream file(input_file); 
    
    if (!file.is_open()) {
      std::cerr << "Error: Could not open the file." << std::endl;
      return 1;
    }
    
    std::vector<rowData> dataList;    
    std::string line;
    
    // Skip the header row
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string timestamp_ms, x_e4, xdot_e3, theta_e4, thetadot_e3, u_e3;
        
        if (std::getline(ss, timestamp_ms, ',') && 
            std::getline(ss, x_e4, ',') &&
            std::getline(ss, xdot_e3, ',') &&
            std::getline(ss, theta_e4, ',') &&
            std::getline(ss, thetadot_e3, ',') &&
            std::getline(ss, u_e3)) {
            
            try {
                float timestamp = std::stof(timestamp_ms);
                float x = std::stof(x_e4);
                float xdot = std::stof(xdot_e3);
                float theta = std::stof(theta_e4);
                float thetadot = std::stof(thetadot_e3);
                float u = std::stof(u_e3);
                
                dataList.push_back(rowData{
                    timestamp / 1000.0f,
                    x / 10000.0f,
                    xdot / 1000.0f,
                    theta / 10000.0f,
                    thetadot / 1000.0f,
                    u / 1000.0f
                });
            }
            
            catch (const std::invalid_argument& e) {
                // Catches rows with text/headers that cannot become numbers
                std::cerr << "Skipping invalid row (not a number): " << line << std::endl;
            }
            catch (const std::out_of_range& e) {
                // Catches numbers too large or small to fit in a float
                std::cerr << "Skipping row (number out of range): " << line << std::endl;
            }
        }         
    }
    file.close();
    
    std::sort(dataList.begin(), dataList.end(), [](const rowData& a, const rowData& b) {
        return a.timestamp < b.timestamp;
    });
    
    rowData x0_raw = dataList[0];
    const std::array<float, 4> x0{x0_raw.x, x0_raw.xdot, x0_raw.theta, x0_raw.thetadot};
    Kalman observer(x0);
    
    std::vector<exportData> result;
    std::array<float, 4> x_hat;
    Matrix P;
    Matrix Kf;
    for (const auto& row: dataList) {
        //std::cout << row.timestamp << " | " << row.x << " | " << row.xdot << " | " << row.theta << " | " << row.thetadot << " | " << row.u << std::endl;
        observer.predict(row.u);
        observer.update({row.x, row.xdot, row.theta, row.thetadot});
        x_hat = observer.getStateEstimate();
        P = observer.P;
        Kf = observer.Kf;
        result.push_back(exportData{
            row.timestamp,
            row.x,
            row.xdot,
            row.theta,
            row.thetadot,
            row.u,
            x_hat[0],
            x_hat[1],
            x_hat[2],
            x_hat[3],
            P[0][0] + P[1][1] + P[2][2] + P[3][3],
            Kf[0][0] + Kf[1][1] + Kf[2][2] + Kf[3][3],
        });
    }
    
    // 1. Find the position of the very last forward slash '/'
    size_t last_slash_idx = input_file.find_last_of("/");
    
    if (last_slash_idx == std::string::npos) {
        std::cerr << "Error: No slash found in path." << std::endl;
        return 1;
    }
    
    // 2. Split the string into the directory path and the filename
    std::string folder_path = input_file.substr(0, last_slash_idx + 1); // Includes the trailing '/'
    std::string filename = input_file.substr(last_slash_idx + 1);      // Just "data-timestamps..."
    
    // 3. Construct the new path by injecting "result-" right before the filename
    std::string new_path = folder_path + "result-" + filename;
    writeCSV(new_path, result);
    
    /*
    const std::array<float, 4> x0{{0.0, 0.0, 0.0, 0.0}};
    Kalman observer(x0);
    
    float u{0.01};
    observer.predict(u);

    std::array<float, 4> z{{0.215, 0.05, 0.9873, -0.05}};
    observer.update(z);
    
    for (int i=0; i<x0.size(); i++) {
        std::cout << "x_hat_" << i << ": " << observer.getStateEstimate()[i] << std::endl;
    }
    */
    return 0;
}