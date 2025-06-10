# Robot Arm Control

Este proyecto permite controlar un brazo robótico usando C++. El programa principal es `./control`, y desde ahí se manejan todas las funciones.

## ¿Qué hace?

- **Graba trayectorias** con `grab_test`. Esto guarda un archivo `.csv` con los movimientos. Es importante usar los nombres correctos si luego se quieren reproducir.
- **Reproduce trayectorias** con `./control`, que tiene tres opciones:
  1. Toma una foto y vuelve a la posición inicial.
  2. Reproduce la trayectoria paso por paso, esperando que presiones `Enter` para avanzar.
  3. Ejecuta automáticamente toda la trayectoria, tomando fotos en intervalos de tiempo definidos por el usuario (en segundos).

---

## Cómo compilar y ejecutar

### Opción 1: Con Docker (recomendado si no quieres instalar nada)

1. Asegúrate de tener Docker instalado.
2. Corre este comando para entrar al contenedor con permisos de USB:

```bash
sudo docker run -it --privileged -v $(pwd):/app -w /app rover bash
```

Dentro del contenedor:

```bash
ls  # Verifica si ya existe la carpeta build
mkdir build  # Si no existe
cd build
cmake ..
make
```
Luego ejecuta normalmente:
```bash
./control
```

### Opción 2: En una máquina virtual o Ubuntu 22.04
Asegúrate de tener instalado lo siguiente:

```bash
sudo apt install cmake build-essential libopencv-dev libserialport-dev libcurl4-openssl-dev
```
Compila el proyecto:

```bash
cd Shadow_Tulip_Arm
cd build
cmake ..
make
```
Ejecuta el programa:

```bash
./control
```
### Extras

grab_test sirve para grabar trayectorias.

test_move es útil para pruebas manuales.

Las fotos se toman automáticamente con la opción 1 y 3 de ./control.

Los archivos .csv con las trayectorias deben tener nombres válidos y conocidos por el sistema para ser reconocidos.


