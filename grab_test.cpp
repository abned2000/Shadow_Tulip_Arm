#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <libserialport.h>
#include <map>
#include <fstream>
#include <mutex>
#include "motor.h"

#define GEAR_RATIO 6.33f
#define MAX_BUFFER_SIZE 1024

std::atomic<bool> g_running(true);
std::atomic<bool> g_recording(false);
std::map<std::string, Motor> g_motors;
std::mutex file_mutex;
std::ofstream csv_file;

std::map<std::string, std::string> logical_serials = {
    {"J1", "FT9MFRIH"},
    {"J2", "FTA7NOUS"},
    {"J3", "FT9MIQY1"}
};

std::map<std::string, int> motor_ids = {
    {"J1", 2},
    {"J2", 2},
    {"J3", 2}
};

void signal_handler(int) {
    std::cout << "\nCtrl+C detectado. Finalizando...\n";
    g_running = false;
    g_recording = false;
}

int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B4000000);
    cfsetispeed(&tty, B4000000);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ECHO | ICANON | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

std::map<std::string, std::string> detect_ports() {
    std::map<std::string, std::string> serial_to_port;
    struct sp_port **ports;

    if (sp_list_ports(&ports) != SP_OK) return serial_to_port;

    for (int i = 0; ports[i] != nullptr; ++i) {
        const char* serial = sp_get_port_usb_serial(ports[i]);
        if (serial) serial_to_port[serial] = sp_get_port_name(ports[i]);
    }

    sp_free_port_list(ports);
    return serial_to_port;
}

void channel_thread(const std::string& joint, const std::string& port, int motor_id) {
    int fd = initialize_serial_port(port.c_str());
    if (fd < 0) {
        std::cerr << "[Error] No se pudo abrir " << port << "\n";
        return;
    }

    uint8_t recv_buffer[MAX_BUFFER_SIZE];

    while (g_running) {
        auto packet = g_motors[joint].createControlPacket(motor_id);
        write(fd, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        g_motors[joint].incrementSendCount();

        ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
        if (bytes_read >= sizeof(Motor::RecvData_t)) {
            auto* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);
            if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                uint16_t crc = crc_ccitt(0x2cbb, reinterpret_cast<uint8_t*>(recv_packet), sizeof(Motor::RecvData_t) - 2);
                if (crc == recv_packet->CRC16) {
                    g_motors[joint].updateFeedback(*recv_packet);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(fd);
}

void menu_loop() {
    std::cout << "\n=== MONITOREO Y GRABACIÓN DE POSICIONES ===\n";
    std::cout << "Comandos:\n"
              << "  s : start grabación\n"
              << "  x : stop grabación\n"
              << "  q : salir\n";

    auto t0 = std::chrono::steady_clock::now();

    while (g_running) {
        char cmd;
        std::cout << "\n> Comando: ";
        std::cin >> cmd;

        switch (cmd) {
            case 's':
                if (!g_recording) {
                    std::lock_guard<std::mutex> lock(file_mutex);
                    csv_file.open("movimientos.csv");
                    csv_file << "tiempo,j1,j2,j3\n";
                    g_recording = true;
                    std::cout << "[Grabación iniciada]\n";
                } else {
                    std::cout << "[Ya está grabando]\n";
                }
                break;
            case 'x':
                if (g_recording) {
                    g_recording = false;
                    std::lock_guard<std::mutex> lock(file_mutex);
                    csv_file.close();
                    std::cout << "[Grabación detenida y archivo guardado]\n";
                } else {
                    std::cout << "[No está grabando]\n";
                }
                break;
            case 'q':
                g_running = false;
                break;
            default:
                std::cout << "[Comando no válido]\n";
        }
    }
}

void monitor_loop() {
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        float j1 = g_motors["J1"].getPosition() / GEAR_RATIO;
        float j2 = g_motors["J2"].getPosition() / GEAR_RATIO;
        float j3 = g_motors["J3"].getPosition() / GEAR_RATIO;

        auto now = std::chrono::steady_clock::now();
        float t = std::chrono::duration<float>(now - t0).count();

        std::cout << "\r[ " << t << " s ] J1: " << j1 << " | J2: " << j2 << " | J3: " << j3 << "       " << std::flush;

        if (g_recording) {
            std::lock_guard<std::mutex> lock(file_mutex);
            if (csv_file.is_open()) {
                csv_file << t << "," << j1 << "," << j2 << "," << j3 << "\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    signal(SIGINT, signal_handler);
    auto ports = detect_ports();

    for (const auto& [joint, serial] : logical_serials) {
        if (ports.find(serial) == ports.end()) {
            std::cerr << "[Error] No se encontró el puerto para " << joint << " (" << serial << ")\n";
            return 1;
        }
    }

    std::vector<std::thread> threads;
    for (const auto& [joint, serial] : logical_serials) {
        std::string port = ports[serial];
        g_motors[joint] = Motor();
        threads.emplace_back(channel_thread, joint, port, motor_ids[joint]);
    }

    std::thread monitor(monitor_loop);
    menu_loop();

    monitor.join();
    for (auto& t : threads) t.join();

    std::cout << "\nFinalizado correctamente.\n";
    return 0;
}
