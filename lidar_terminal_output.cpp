#include <iostream>
#include <chrono>
#include <limits>
#include <thread>
#include "include/ldlidar_driver/ldlidar_driver_linux.h"

void printClosestInQuadrants(const ldlidar::Points2D& points, int scan_count) {
    // Initialize min distances to a large number and angles to -1 for each quadrant
    double min_dist[4] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    double min_angle[4] = {-1, -1, -1, -1};

    for (const auto& point : points) {
        if (point.distance == 0) continue; // Skip points with zero distance

        int quadrant = -1;
        if (point.angle >= 0 && point.angle < 90) {
            quadrant = 0;
        } else if (point.angle >= 90 && point.angle < 180) {
            quadrant = 1;
        } else if (point.angle >= 180 && point.angle < 270) {
            quadrant = 2;
        } else if (point.angle >= 270 && point.angle < 360) {
            quadrant = 3;
        }

        if (quadrant != -1 && point.distance < min_dist[quadrant]) {
            min_dist[quadrant] = point.distance;
            min_angle[quadrant] = point.angle;
        }
    }

    // Clear the terminal (Linux-specific, use "cls" for Windows if needed)
    std::cout << "\033[2J\033[1;1H"; // ANSI escape sequence to clear screen and move cursor to top-left

    // Print results for each quadrant
    std::cout << "----- Scan " << scan_count << " -----" << std::endl;
    for (int i = 0; i < 4; ++i) {
        std::cout << "Quadrant " << (i + 1) << " (" << i * 90 << "° - " << (i + 1) * 90 << "°): ";
        if (min_angle[i] != -1) {
            std::cout << "Closest object at angle " << min_angle[i] << " deg, distance " << min_dist[i] << " mm" << std::endl;
        } else {
            std::cout << "No valid data" << std::endl;
        }
    }
}

int main() {
    // Specify your LiDAR model and serial port
    ldlidar::LDType lidar_type = ldlidar::LDType::LD_06; // Example: LD06, adjust as needed
    std::string serial_port = "/dev/ttyUSB0"; // Adjust to your serial port
    uint32_t baudrate = 230400; // Default baudrate for LD06, adjust per model

    // Create LiDAR driver instance
    ldlidar::LDLidarDriverLinuxInterface* lidar_driver = new ldlidar::LDLidarDriverLinuxInterface();

    // Register timestamp callback (required for data processing)
    lidar_driver->RegisterGetTimestampFunctional(std::bind(
        []() -> uint64_t {
            auto now = std::chrono::system_clock::now();
            return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        }));

    // Enable noise filtering (optional, recommended for cleaner data)
    lidar_driver->EnablePointCloudDataFilter(true);

    // Connect to the LiDAR
    if (!lidar_driver->Connect(lidar_type, serial_port, baudrate, ldlidar::COMM_SERIAL_MODE)) {
        std::cerr << "Failed to connect to LiDAR on " << serial_port << std::endl;
        delete lidar_driver;
        return 1;
    }

    // Wait for LiDAR communication to stabilize
    if (!lidar_driver->WaitLidarComm(1000)) {
        std::cerr << "LiDAR communication timeout" << std::endl;
        lidar_driver->Disconnect();
        delete lidar_driver;
        return 1;
    }

    // Start the LiDAR
    if (!lidar_driver->Start()) {
        std::cerr << "Failed to start LiDAR" << std::endl;
        lidar_driver->Disconnect();
        delete lidar_driver;
        return 1;
    }

    std::cout << "LiDAR started successfully. Press Ctrl+C to stop." << std::endl;

    // Main loop to retrieve and print data
    int scan_count = 0;
    while (lidar_driver->Ok()) {
        ldlidar::Points2D points;
        ldlidar::LidarStatus status = lidar_driver->GetLaserScanData(points, 1000); // 1-second timeout
        switch (status) {
            case ldlidar::LidarStatus::NORMAL:
                printClosestInQuadrants(points, scan_count);
                scan_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms delay
                break;
            case ldlidar::LidarStatus::DATA_TIME_OUT:
                std::cerr << "LiDAR data timeout" << std::endl;
                break;
            case ldlidar::LidarStatus::DATA_WAIT:
                // Waiting for data, continue loop
                break;
            case ldlidar::LidarStatus::STOP:
                std::cerr << "LiDAR stopped" << std::endl;
                break;
            default:
                std::cerr << "LiDAR error, code: " << (int)lidar_driver->GetLidarErrorCode() << std::endl;
                break;
        }
    }

    // Cleanup
    lidar_driver->Stop();
    lidar_driver->Disconnect();
    delete lidar_driver;
    return 0;
}