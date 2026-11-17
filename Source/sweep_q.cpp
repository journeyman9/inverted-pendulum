#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
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
    float u_cl;
    float x_hat_0;
    float x_hat_1;
    float x_hat_2;
    float x_hat_3;
    float trace_P;
    float trace_Kf;

    std::string to_csv_line() const {
        std::stringstream ss;
        ss << timestamp << "," << x << "," << xdot << "," << theta << "," << thetadot << "," << u << "," << u_cl << ","
           << x_hat_0 << "," << x_hat_1 << "," << x_hat_2 << "," << x_hat_3 << "," << trace_P << "," << trace_Kf;
        return ss.str();
    }
};

struct QConfig {
    const char* label;
    float q0, q1, q2, q3;
};

void writeCSV(const std::string& filepath, const std::vector<exportData>& dataset) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filepath << std::endl;
        return;
    }
    file << "timestamp,x,xdot,theta,thetadot,u,u_cl,x_hat_0,x_hat_1,x_hat_2,x_hat_3,trace_P,trace_Kf\n";
    for (const auto& row: dataset) {
        file << row.to_csv_line() << "\n";
    }
    file.close();
    std::cout << "Wrote " << dataset.size() << " rows to " << filepath << std::endl;
}

struct SweepResult {
    float rms;
    float max_abs;
    bool diverged;
    std::vector<exportData> data;
};

SweepResult runTrial(const std::vector<rowData>& dataList, float position_set, float angle_set,
                     const QConfig& cfg, const float K[4], bool keep_data) {
    const std::array<float, 4> x0{0.0f, dataList[0].xdot, 0.0f, dataList[0].thetadot};
    Kalman observer(x0);
    observer.Q = {{
        {{cfg.q0, 0.0f, 0.0f, 0.0f}},
        {{0.0f, cfg.q1, 0.0f, 0.0f}},
        {{0.0f, 0.0f, cfg.q2, 0.0f}},
        {{0.0f, 0.0f, 0.0f, cfg.q3}}
    }};

    SweepResult res;
    res.diverged = false;
    float sum_sq = 0.0f;
    res.max_abs = 0.0f;

    for (const auto& row: dataList) {
        observer.update({row.x - position_set, row.xdot, row.theta - angle_set, row.thetadot});
        std::array<float, 4> xh = observer.getStateEstimate();

        float u_cl_voltage = -1.0f * (
            K[0] * xh[0] + K[1] * xh[1] + K[2] * xh[2] + K[3] * xh[3]
        );
        float u_cl_pwm = u_cl_voltage * (1600.0f / 24.0f);
        if (u_cl_pwm > 1600.0f) u_cl_pwm = 1600.0f;
        else if (u_cl_pwm < -1600.0f) u_cl_pwm = -1600.0f;

        float err = u_cl_voltage - row.u;
        sum_sq += err * err;
        if (std::abs(err) > res.max_abs) res.max_abs = std::abs(err);
        if (std::abs(u_cl_voltage) > 24.0f) res.diverged = true;

        if (keep_data) {
            res.data.push_back(exportData{
                row.timestamp, row.x, row.xdot, row.theta, row.thetadot,
                row.u, u_cl_voltage,
                xh[0] + position_set, xh[1], xh[2] + angle_set, xh[3],
                observer.P[0][0] + observer.P[1][1] + observer.P[2][2] + observer.P[3][3],
                observer.Kf[0][0] + observer.Kf[1][1] + observer.Kf[2][2] + observer.Kf[3][3],
            });
        }

        observer.predict(u_cl_pwm * (24.0f / 1600.0f));
    }

    res.rms = std::sqrt(sum_sq / dataList.size());
    return res;
}

int main() {
    std::string input_file = "./logged_data/kalman/2026-11-15-21-58-no-kalman.csv";
    std::ifstream file(input_file);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return 1;
    }

    std::vector<rowData> dataList;
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string timestamp_ms, x_e4, xdot_e3, theta_e3, thetadot_e3, u_e3;

        if (std::getline(ss, timestamp_ms, ',') &&
            std::getline(ss, x_e4, ',') &&
            std::getline(ss, xdot_e3, ',') &&
            std::getline(ss, theta_e3, ',') &&
            std::getline(ss, thetadot_e3, ',') &&
            std::getline(ss, u_e3)) {

            try {
                dataList.push_back(rowData{
                    std::stof(timestamp_ms) / 1000.0f,
                    std::stof(x_e4) / 10000.0f,
                    std::stof(xdot_e3) / 1000.0f,
                    std::stof(theta_e3) / 1000.0f,
                    std::stof(thetadot_e3) / 1000.0f,
                    std::stof(u_e3) / 1000.0f
                });
            }
            catch (...) {}
        }
    }
    file.close();

    std::sort(dataList.begin(), dataList.end(), [](const rowData& a, const rowData& b) {
        return a.timestamp < b.timestamp;
    });

    float position_set = dataList[0].x;
    float angle_set = dataList[0].theta;
    const float K[4] = {-25.082511f, -11.15f, 30.379604f, 3.00f};

    QConfig configs[] = {
        // Original
        {"original",         0.0001f,  0.0001f,  0.0005f,  0.01f},
        // Coarse sweep
        {"vel_10x",          0.0001f,  0.001f,   0.0005f,  0.1f},
        {"vel_100x",         0.0001f,  0.01f,    0.0005f,  1.0f},
        {"vel_1000x",        0.0001f,  0.1f,     0.0005f,  10.0f},
        {"all_10x",          0.001f,   0.001f,   0.005f,   0.1f},
        {"all_100x",         0.01f,    0.01f,    0.05f,    1.0f},
        {"all_1000x",        0.1f,     0.1f,     0.5f,     10.0f},
        // Isolate which velocity channel matters
        {"xdot_only_100x",   0.0001f,  0.01f,    0.0005f,  0.01f},
        {"thdot_only_100x",  0.0001f,  0.0001f,  0.0005f,  1.0f},
        // Fine sweep around xdot boundary
        {"xd_20x",           0.0001f,  0.002f,   0.0005f,  0.01f},
        {"xd_30x",           0.0001f,  0.003f,   0.0005f,  0.01f},
        {"xd_50x",           0.0001f,  0.005f,   0.0005f,  0.01f},
        {"xd_70x",           0.0001f,  0.007f,   0.0005f,  0.01f},
        // Practical candidates for hardware
        {"match_R",          0.0001f,  0.01f,    0.0036f,  0.09f},
        {"exceed_R",         0.001f,   0.1f,     0.01f,    0.5f},
        {"conservative",     0.001f,   0.05f,    0.005f,   0.25f},
    };
    int num_configs = sizeof(configs) / sizeof(configs[0]);

    std::cout << std::endl;
    std::cout << "Q Sweep: closed-loop stability (u_cl vs u_raw from no-kalman data)" << std::endl;
    std::cout << "=====================================================================" << std::endl;
    printf("%-18s  %8s %8s %8s %8s  %8s %8s %7s\n",
           "config", "Q_x", "Q_xd", "Q_th", "Q_thd", "rms_u", "max_u", "status");
    std::cout << "---------------------------------------------------------------------" << std::endl;

    int best_idx = -1;
    float best_rms = 1e9f;

    for (int c = 0; c < num_configs; c++) {
        SweepResult res = runTrial(dataList, position_set, angle_set, configs[c], K, false);

        printf("%-18s  %8.4f %8.4f %8.4f %8.4f  %8.3f %8.3f %7s\n",
               configs[c].label,
               configs[c].q0, configs[c].q1, configs[c].q2, configs[c].q3,
               res.rms, res.max_abs,
               res.diverged ? "DIVERGE" : "ok");

        if (!res.diverged && res.rms < best_rms) {
            best_rms = res.rms;
            best_idx = c;
        }
    }

    std::cout << std::endl;

    if (best_idx >= 0) {
        const auto& best = configs[best_idx];
        std::cout << "Best config: " << best.label << " (rms=" << best_rms << ")" << std::endl;
        std::cout << "  Q = diag(" << best.q0 << ", " << best.q1 << ", " << best.q2 << ", " << best.q3 << ")" << std::endl;

        SweepResult best_res = runTrial(dataList, position_set, angle_set, best, K, true);

        size_t last_slash_idx = input_file.find_last_of("/");
        std::string folder_path = input_file.substr(0, last_slash_idx + 1);
        std::string new_path = folder_path + "result-sweep-best.csv";
        writeCSV(new_path, best_res.data);
    } else {
        std::cout << "All configs diverged!" << std::endl;
    }

    return 0;
}
