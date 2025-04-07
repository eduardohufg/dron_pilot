# Dron_pilot - Prueba tecnica

### Descripción:

Se necesita que un dron regrese de manera autonoma a un punto determinado para cargarse cuando tiene bateria baja, esto se puede hacer referenciando al dorn a un punto y orientarlo, esto se logra mediante la deteccion, aproximacion y centrado a un Aruco. Este trabajo muestra un algoritmo para lograr el objetivo con ayuda de la simualcion de PX4, ROS2 y la implementacion de una interfaz grafica

# Instrucciones para correr el proyecto
### Pre-requisitos:

para correr este proyecto es necesario tener instalado ROS2 Humble, git, python 3.10 o superior y NodeJs

1. *Node.js*: Puedes descargarlo desde el siguiente enlace:
   [Descargar Node.js](https://nodejs.org/en/download/package-manager)

2. *Python 3.10 o superior*: Puedes descargar la versión correcta desde el siguiente enlace:
   [Descargar Python](https://www.python.org/downloads/)

3. *Git*: Asegúrate de tenerlo instalado. Puedes obtenerlo desde el siguiente enlace:
   [Descargar Git](https://git-scm.com/downloads)

4. *Ros Humble*: Debes tener dispositivo con ubuntu 22.04 o alguna distribucion de linux que soporte ROS
    [Descargar ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

### Instalacion y setup de la simulacion:

Abre una terminal y corre los siguientes comandos:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
