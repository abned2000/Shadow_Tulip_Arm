#include <iostream>
#include <map>
#include <atomic>
#include <csignal>
#include <thread>
#include <vector>
#include <tuple>
#include <string>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <libserialport.h>
#include "motor.h"
#include <opencv2/opencv.hpp>

#define GEAR_RATIO 6.33f

std::atomic<bool> g_running(true);
std::map<std::string, Motor> g_motors;

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
    std::cout << "\n[SALIDA] Ctrl+C detectado.\n";
    g_running = false;
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

void motor_thread(const std::string& joint, const std::string& port, int motor_id) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "[Error] No se pudo abrir " << port << "\n";
        return;
    }

    uint8_t buffer[1024];

    while (g_running) {
        auto packet = g_motors[joint].createControlPacket(motor_id);
        write(fd, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read >= sizeof(Motor::RecvData_t)) {
            auto* recv_packet = reinterpret_cast<Motor::RecvData_t*>(buffer);
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

bool reproducirCSV(const std::string& nombre_archivo, const std::string& modo) {
    std::ifstream file(nombre_archivo);
    if (!file.is_open()) {
        std::cerr << "[Error] No se pudo abrir el archivo " << nombre_archivo << "\n";
        return false;
    }

    std::string line;
    std::getline(file, line);
    std::vector<std::tuple<float, float, float, float>> movimientos;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;
        while (std::getline(ss, item, ',')) tokens.push_back(item);
        if (tokens.size() < 4) continue;

        float tiempo = std::stof(tokens[0]);
        float j1 = std::stof(tokens[1]);
        float j2 = std::stof(tokens[2]);
        float j3 = std::stof(tokens[3]);

        movimientos.emplace_back(tiempo, j1, j2, j3);
    }

    for (size_t i = 0; i < movimientos.size() && g_running; ++i) {
        auto [t, j1, j2, j3] = movimientos[i];
        float torque = 0.03f / GEAR_RATIO;
        float vel    = 0.02f * GEAR_RATIO;
        float kp     = 8.0f / (GEAR_RATIO * GEAR_RATIO);
        float kd     = 1.5f / (GEAR_RATIO * GEAR_RATIO);

        g_motors["J1"].setControlParams(torque, vel, j1 * GEAR_RATIO, kp, kd);
        g_motors["J2"].setControlParams(torque, vel, j2 * GEAR_RATIO, kp, kd);
        g_motors["J3"].setControlParams(torque, vel, j3 * GEAR_RATIO, kp, kd);

        std::cout << "Tiempo: " << t << "s -> J1: " << j1 << ", J2: " << j2 << ", J3: " << j3 << "\n";

        if (modo == "l") {
            std::cout << "Presiona Enter para continuar...";
            std::cin.get();
        } else if (i + 1 < movimientos.size()) {
            float dt = std::get<0>(movimientos[i + 1]) - t;
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 400)));
        }
    }
    std::cout << "Terminado";
    return true;
}

void menu_principal() {
    while (g_running) {
        std::string opcion;

        std::cout << "\n🌟 === MENÚ DE CONTROL === 🌟\n";
        std::cout << "1️⃣  📸 Tomate una foto\n";
        std::cout << "2️⃣  📷 Testeo (Warning)\n";
        std::cout << "3️⃣  🔒 Bloquear motores\n";
        std::cout << "4️⃣  🔓 Desbloquear motores\n";
        std::cout << "📝 Seleccione una opción: ";

        std::cin >> opcion;
        std::cin.ignore();

        if (opcion == "1" || opcion == "2") {
            std::string modo = (opcion == "1") ? "c" : "l";

            if (!reproducirCSV("camera_start.csv", modo)) continue;

            cv::VideoCapture cam(0);
            if (!cam.isOpened()) {
                std::cerr << "[Error] No se pudo acceder a la cámara.\n";
            } else {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                cv::Mat foto; cam >> foto;
                if (!foto.empty()) {
                    auto t = std::time(nullptr);
                    std::tm tm; localtime_r(&t, &tm);
                    char buffer[100];
                    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm);


                    cv::Mat foto_rotada;
                    cv::rotate(foto, foto_rotada, cv::ROTATE_90_CLOCKWISE);

                    // Obtener fecha y hora actual
                    auto now = std::chrono::system_clock::now();
                    std::time_t tiempo_actual = std::chrono::system_clock::to_time_t(now);
                    std::tm* tiempo_tm = std::localtime(&tiempo_actual);

                    std::ostringstream nombre_archivo;
                    nombre_archivo << "foto_"
                                << std::put_time(tiempo_tm, "%Y%m%d_%H%M%S")
                                << ".jpg";

                    std::string nombre = nombre_archivo.str();    
                    cv::imwrite(nombre, foto_rotada);
                    std::cout << "✅ Imagen guardada como: " << nombre << std::endl;
                }
                cam.release();
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            reproducirCSV("camera_home.csv", modo);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "Motores Liberados";
            for (auto& [joint, motor] : g_motors) {
                motor.setControlParams(0, 0, 0, 0, 0);
            }
            
        }
        else if (opcion == "3") {
            std::cout << "\n[INFO] Bloqueando motores...\n";

            for (auto& [joint, motor] : g_motors) {
                    float pos = motor.getPosition() / GEAR_RATIO;
                    std::cout << "Posición: " << pos << std::endl;
                    motor.setControlParams(0, 0, pos * GEAR_RATIO, 15.0f, 0);
                    
                }

        }

        else if (opcion == "4") {
            std::cout << "\n[INFO] Desbloqueando motores...\n";
            for (auto& [joint, motor] : g_motors) {
                motor.setControlParams(0, 0, 0, 0, 0);
            }

        }
        else {
            std::cout << "[Error] Opción no válida.\n";
        }
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
        g_motors[joint] = Motor();
        threads.emplace_back(motor_thread, joint, ports[serial], motor_ids[joint]);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    menu_principal();

    for (auto& t : threads) t.join();
    std::cout << "Sistema finalizado.\n";
    return 0;
}
