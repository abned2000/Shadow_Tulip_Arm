#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <csignal>
#include <libserialport.h>
#include <map>
#include <cmath>
#include "motor.h"
#include <array>

#define GEAR_RATIO 6.33f
#define MAX_BUFFER_SIZE 1024

std::atomic<bool> g_running(true);
std::map<std::string, Motor> g_motors;

// Seriales reales de tus interfaces USB
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

// Límites seguros por articulación (en radianes)
std::map<std::string, float> joint_min = {
    {"J1", -1.57f}, {"J2", -1.0f}, {"J3", -0.5f}
};
std::map<std::string, float> joint_max = {
    {"J1", 1.57f}, {"J2", 1.5f}, {"J3", 1.0f}
};

std::map<std::string, float> pos_ref;

const float paso_pequeno = 0.01f;
const float paso_grande = 0.05f;
float paso_actual = paso_pequeno;
float k_pos = 5.0f;

// Manejo de señal para cerrar correctamente
void signal_handler(int) {
    std::cout << "\nCtrl+C detectado. Saliendo...\n";
    g_running = false;
}

// Inicializa puerto serial
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

// Detecta y asigna puertos conectados
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

// Hilo por canal para comunicación continua
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
// Cinematica para el brazo


struct Vec3 {
    float x, y, z;
};

// Convierte grados a radianes
float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

// Multiplica dos matrices 4x4
std::array<std::array<float, 4>, 4> mat_mult(const std::array<std::array<float, 4>, 4>& A,
                                             const std::array<std::array<float, 4>, 4>& B) {
    std::array<std::array<float, 4>, 4> result = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                result[i][j] += A[i][k] * B[k][j];
    return result;
}

// Construye matriz DH individual
std::array<std::array<float, 4>, 4> dh_matrix(float theta_deg, float d, float a, float alpha_deg) {
    float theta = deg2rad(theta_deg);
    float alpha = deg2rad(alpha_deg);
    return {{
        { cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta) },
        { sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta) },
        { 0, sin(alpha), cos(alpha), d },
        { 0, 0, 0, 1 }
    }};
}

// Calcula la posición del extremo
Vec3 forward_kinematics(float theta1_deg, float theta2_deg, float theta3_deg) {
    // Parámetros DH de tu tabla (en metros)
    auto A1 = dh_matrix(theta1_deg, 0.052, 0.0, 0.0);
    auto A2 = dh_matrix(theta2_deg, 0.042, 0.119, 90.0);
    auto A3 = dh_matrix(theta3_deg, -0.09, 0.15, 0.0);

    auto T1 = mat_mult(A1, A2);
    auto T = mat_mult(T1, A3);

    return { T[0][3], T[1][3], T[2][3] };  // x, y, z
}

// Grabar Movimientos
struct JointPose {
    float j1, j2, j3;
    float timestamp;  // tiempo relativo desde el inicio
};

std::vector<JointPose> recorded_poses;
bool is_recording = false;
std::chrono::steady_clock::time_point start_time;

void start_recording() {
    recorded_poses.clear();
    is_recording = true;
    start_time = std::chrono::steady_clock::now();
    std::cout << "[Grabación] Iniciada...\n";
}

void stop_recording() {
    is_recording = false;
    std::cout << "[Grabación] Finalizada. Se guardaron " << recorded_poses.size() << " poses.\n";
}

void record_current_pose() {
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - start_time).count();

    JointPose pose;
    pose.j1 = pos_ref["J1"];
    pose.j2 = pos_ref["J2"];
    pose.j3 = pos_ref["J3"];
    pose.timestamp = elapsed;
    recorded_poses.push_back(pose);
}

void play_recording() {
    if (recorded_poses.empty()) {
        std::cout << "[Reproducción] No hay movimientos grabados.\n";
        return;
    }

    std::cout << "[Reproducción] Ejecutando secuencia...\n";
    auto t0 = std::chrono::steady_clock::now();

    for (size_t i = 0; i < recorded_poses.size(); ++i) {
        JointPose& pose = recorded_poses[i];

        // Esperar hasta que llegue el tiempo adecuado
        float wait_time = pose.timestamp - std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
        if (wait_time > 0)
            std::this_thread::sleep_for(std::chrono::duration<float>(wait_time));

        g_motors["J1"].setControlParams(0, 0, pose.j1 * GEAR_RATIO, 30.0f / (GEAR_RATIO * GEAR_RATIO), 3.0f / (GEAR_RATIO * GEAR_RATIO));
        g_motors["J2"].setControlParams(0, 0, pose.j2 * GEAR_RATIO, 30.0f / (GEAR_RATIO * GEAR_RATIO), 3.0f / (GEAR_RATIO * GEAR_RATIO));
        g_motors["J3"].setControlParams(0, 0, pose.j3 * GEAR_RATIO, 30.0f / (GEAR_RATIO * GEAR_RATIO), 3.0f / (GEAR_RATIO * GEAR_RATIO));

        std::cout << "[Pose " << i+1 << "] j1=" << pose.j1 << ", j2=" << pose.j2 << ", j3=" << pose.j3 << "\n";
    }

    std::cout << "[Reproducción] Finalizada.\n";
}


// Menú de control CLI
void menu_control(const std::map<std::string, std::string>& ports) {
    std::cout << "\n=== CONTROL DE BRAZO ROBÓTICO ===\n";
    std::cout << "Comandos:\n"
              << " 1/2: +J1 / -J1\n"
              << " 3/4: +J2 / -J2\n"
              << " 5/6: +J3 / -J3\n"
              << " p : cambiar tamaño de paso\n"
              << " s : mostrar posiciones actuales\n"
              << " x : detener motores\n"
              << " g : grabar movimientos\n"
              << " r : reproducir secuencia grabada\n"
              << " q : salir\n";

    for (const auto& [joint, motor] : g_motors)
        pos_ref[joint] = motor.getPosition() / GEAR_RATIO;

    char cmd;
    while (g_running) {
        std::cout << "\n> Comando: ";
        std::cin >> cmd;

        std::string joint;
        float signo = 0.0f;

        switch (cmd) {
            case '1': joint = "J1"; signo = +1.0f; break;
            case '2': joint = "J1"; signo = -1.0f; break;
            case '3': joint = "J2"; signo = +1.0f; break;
            case '4': joint = "J2"; signo = -1.0f; break;
            case '5': joint = "J3"; signo = +1.0f; break;
            case '6': joint = "J3"; signo = -1.0f; break;
            case 'p':
                paso_actual = (paso_actual == paso_pequeno) ? paso_grande : paso_pequeno;
                std::cout << "Paso actual: " << paso_actual << " rad\n";
                continue;
            case 's': {
                for (auto& [j, motor] : g_motors)
                    std::cout << j << ": " << motor.getPosition() / GEAR_RATIO << " rad\n";

                float t1_deg = g_motors["J1"].getPosition() / GEAR_RATIO * 180.0f / M_PI;
                float t2_deg = g_motors["J2"].getPosition() / GEAR_RATIO * 180.0f / M_PI;
                float t3_deg = g_motors["J3"].getPosition() / GEAR_RATIO * 180.0f / M_PI;

                Vec3 pos = forward_kinematics(t1_deg, t2_deg, t3_deg);
                std::cout << "Posición estimada del efector: "
                        << "(x=" << pos.x * 1000 << " mm, y=" << pos.y * 1000
                        << " mm, z=" << pos.z * 1000 << " mm)\n";
                break;
            }


            case 'x':
                for (auto& [j, motor] : g_motors) {
                    float pos = motor.getPosition() / GEAR_RATIO;
                    motor.setControlParams(0, 0, pos * GEAR_RATIO, k_pos, 0);
                }
                std::cout << "Motores detenidos.\n";
                continue;
            case 'g':
                if (!is_recording) {
                    start_recording();
                } else {
                    stop_recording();
                }
                continue;

            case 'r':
                play_recording();
                continue;

            case 'q':
                g_running = false;
                return;
            default:
                std::cout << "Comando inválido.\n";
                continue;
        }

        if (!joint.empty()) {

            if (is_recording) {
                record_current_pose();
            }

            float nueva_pos = pos_ref[joint] + signo * paso_actual;
            if (nueva_pos < joint_min[joint] || nueva_pos > joint_max[joint]) {
                std::cout << "[Advertencia] Movimiento fuera de límites.\n";
                continue;
            }

            pos_ref[joint] = nueva_pos;
            g_motors[joint].setControlParams(
                0, 0, nueva_pos * GEAR_RATIO,
                30.0f / (GEAR_RATIO * GEAR_RATIO),
                3.0f / (GEAR_RATIO * GEAR_RATIO)
            );
            std::cout << joint << " → Nueva posición: " << nueva_pos << " rad\n";
        }
    }
}

int main() {
    signal(SIGINT, signal_handler);

    auto ports = detect_ports();

    // Verificar que todos los puertos están conectados
    for (const auto& [joint, serial] : logical_serials) {
        if (ports.find(serial) == ports.end()) {
            std::cerr << "[Error] No se encontró el puerto para " << joint << " (" << serial << ")\n";
            return 1;
        }
    }

    // Inicializar motores y lanzar hilos
    std::vector<std::thread> threads;
    for (const auto& [joint, serial] : logical_serials) {
        std::string port = ports[serial];
        g_motors[joint] = Motor();
        threads.emplace_back(channel_thread, joint, port, motor_ids[joint]);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    menu_control(ports);

    for (auto& t : threads) t.join();
    std::cout << "Sistema finalizado correctamente.\n";
    return 0;
}

