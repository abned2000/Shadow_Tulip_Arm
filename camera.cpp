#include <opencv2/opencv.hpp>
#include <stdio.h>

int main() {
    cv::VideoCapture cap(0); // Abre la cámara predeterminada

    if (!cap.isOpened()) {
        printf("No se pudo abrir la cámara.\n");
        return -1;
    }

    cv::Mat frame;
    int foto_num = 0;

    printf("Presiona 's' para tomar una foto, 'q' para salir.\n");

    while (true) {
        cap >> frame; // Captura un frame

        if (frame.empty()) {
            printf("Error al capturar la imagen.\n");
            break;
        }

        cv::imshow("Camara", frame); // Muestra el frame en una ventana

        char c = (char)cv::waitKey(30); // Espera una tecla

        if (c == 'q') {
            break; // Salir
        } else if (c == 's') {
            char nombre[100];
            sprintf(nombre, "foto_%d.jpg", foto_num++);
            cv::imwrite(nombre, frame); // Guarda la imagen
            printf("Foto guardada como %s\n", nombre);
        }
    }

    cap.release(); // Libera la cámara
    cv::destroyAllWindows(); // Cierra la ventana

    return 0;
}
