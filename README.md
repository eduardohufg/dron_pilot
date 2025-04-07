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

Abre una terminal y corre los siguientes comandos para instalar el simulador:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

Abre otra terminal y corre los siguientes comandos para instalar el agente de comunicacion

```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

En otra terminal corre los siguinetes comandos para configurar el QGroundControl necesario para la simualcion

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

Despues instala la siguiente imagen:

*Download*:
   [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage)

Probablemente se descargo en la carpeta Downloads, por lo que dirijete a esa carpera o a la carpeta donde se haya guardado la descarga y dale permisos de ejecucion

```bash
cd ./Downloads
chmod +x ./QGroundControl.AppImage
```