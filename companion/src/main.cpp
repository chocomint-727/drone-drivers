#include "raylib.h"
#include "raymath.h"
#include "drone_protocol.hpp"

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

// Thread-safe shared attitude state
std::mutex attitudeMutex;
AttitudePacket currentAttitude = {0xAA, 0x55, 1.0f, 0.0f, 0.0f, 0.0f};
std::atomic<bool> serialRunning(true);
std::atomic<bool> packetReceived(false);

void serialReaderThread(const std::string& portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Failed to open serial port: " << portName << std::endl;
        return;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(fd);
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 1;            // read blocks for at least 1 byte
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(fd);
        return;
    }

    std::cout << "Connected to serial port " << portName << " at 115200 baud." << std::endl;

    uint8_t readBuffer[256];
    uint8_t packetBuffer[sizeof(AttitudePacket)];
    size_t packetIndex = 0;

    while (serialRunning) {
        int n = read(fd, readBuffer, sizeof(readBuffer));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                uint8_t byte = readBuffer[i];
                if (packetIndex == 0) {
                    if (byte == 0xAA) {
                        packetBuffer[packetIndex++] = byte;
                    }
                } else if (packetIndex == 1) {
                    if (byte == 0x55) {
                        packetBuffer[packetIndex++] = byte;
                    } else {
                        packetIndex = (byte == 0xAA) ? 1 : 0;
                        if (packetIndex == 1) packetBuffer[0] = 0xAA;
                    }
                } else {
                    packetBuffer[packetIndex++] = byte;
                    if (packetIndex == sizeof(AttitudePacket)) {
                        AttitudePacket pkt;
                        std::memcpy(&pkt, packetBuffer, sizeof(AttitudePacket));
                        
                        {
                            std::lock_guard<std::mutex> lock(attitudeMutex);
                            currentAttitude = pkt;
                        }
                        packetReceived = true;
                        packetIndex = 0;
                    }
                }
            }
        }
    }

    close(fd);
}

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyACM0";
    if (argc > 1) {
        port = argv[1];
    }

    // Start serial reader thread
    std::thread reader(serialReaderThread, port);

    // Initialize Raylib window
    const int screenWidth = 1024;
    const int screenHeight = 768;
    InitWindow(screenWidth, screenHeight, "Drone Attitude Visualizer");
    SetTargetFPS(60);

    // Define Camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 3.0f, 3.0f, 3.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 0.0f, 1.0f }; // Z-up
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        // Camera is fixed; only the drone frame rotates based on telemetry.

        // Get latest attitude quaternion safely
        Quaternion q;
        {
            std::lock_guard<std::mutex> lock(attitudeMutex);
            // Raylib Quaternion: {x, y, z, w} vs glm: {w, x, y, z}
            q = (Quaternion){ currentAttitude.x, currentAttitude.y, currentAttitude.z, currentAttitude.w };
        }
        q = QuaternionNormalize(q);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        // Draw reference grid and axes
        DrawGrid(10, 1.0f);
        DrawLine3D((Vector3){0,0,0}, (Vector3){2,0,0}, LIGHTGRAY);
        DrawLine3D((Vector3){0,0,0}, (Vector3){0,2,0}, LIGHTGRAY);
        DrawLine3D((Vector3){0,0,0}, (Vector3){0,0,2}, LIGHTGRAY);

        // Draw Drone Body Frame Vectors rotated by quaternion q
        // Base vectors in body frame
        Vector3 localX = { 1.5f, 0.0f, 0.0f };
        Vector3 localY = { 0.0f, 1.5f, 0.0f };
        Vector3 localZ = { 0.0f, 0.0f, 1.5f };

        Vector3 worldX = Vector3RotateByQuaternion(localX, q);
        Vector3 worldY = Vector3RotateByQuaternion(localY, q);
        Vector3 worldZ = Vector3RotateByQuaternion(localZ, q);

        // Draw axes (X: Red, Y: Green, Z: Blue)
        DrawLine3D((Vector3){0,0,0}, worldX, RED);
        DrawSphere(worldX, 0.08f, RED);

        DrawLine3D((Vector3){0,0,0}, worldY, GREEN);
        DrawSphere(worldY, 0.08f, GREEN);

        DrawLine3D((Vector3){0,0,0}, worldZ, BLUE);
        DrawSphere(worldZ, 0.08f, BLUE);

        // Draw a simple wireframe cube representing the drone body
        // We can rotate a cube or draw a central body box
        // (For simplicity, axes and spheres show orientation clearly)

        EndMode3D();

        // Draw HUD overlay
        DrawText("Drone Orientation Viewer", 20, 20, 20, DARKGRAY);
        DrawText(TextFormat("Serial Port: %s", port.c_str()), 20, 50, 14, DARKGRAY);
        DrawText(TextFormat("Packet Status: %s", packetReceived ? "Receiving" : "Waiting for packets..."), 20, 75, 14, packetReceived ? GREEN : RED);

        AttitudePacket attCopy;
        {
            std::lock_guard<std::mutex> lock(attitudeMutex);
            attCopy = currentAttitude;
        }

        DrawText(TextFormat("Quat W: %.3f", attCopy.w), 20, 110, 16, BLACK);
        DrawText(TextFormat("Quat X: %.3f", attCopy.x), 20, 135, 16, BLACK);
        DrawText(TextFormat("Quat Y: %.3f", attCopy.y), 20, 160, 16, BLACK);
        DrawText(TextFormat("Quat Z: %.3f", attCopy.z), 20, 185, 16, BLACK);

        EndDrawing();
    }

    serialRunning = false;
    if (reader.joinable()) {
        reader.join();
    }

    CloseWindow();
    return 0;
}
