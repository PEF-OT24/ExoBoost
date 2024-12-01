# Compilación de App
## Set Up

### Construido con: ⚙️

* [Python 3.12](https://www.python.org/downloads/release/python-3125/) - Lenguaje de programación principal.
* [KivyMD](https://kivymd.readthedocs.io/en/latest/) - Framework usado para la app móvil.
* [Java JDK 17](https://techkrowd.com/programacion/java/como-instalar-jdk-17-en-windows-10-y-11/) - Lenguaje de programación usado para interactuar con Android SDK.
* [Android Developer Tools (Android SKD)](https://developer.android.com/tools?hl=es-419) - Herramientas de Android para el desarrollo de servicio BLE.
* [Android Developer Tools (Android NDK)](https://developer.android.com/ndk/downloads?hl=es-419) - Herramientas auxiliares de Android para acceder a la API de android.
* [Buildozer 1.5.0](https://pypi.org/project/buildozer/) - Usado para generar apk con python.
* [Energia IDE](https://energia.nu) - Usado para programación de Tiva C.
* [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) - Usado para la programación de la ESP32 en C++.

#### El diseño de la aplicación se puede modificar tanto en Windows como en Ubuntu puesto que solo depende de python. Para compilar la aplicación y generar las dependencias de Java se utiliza un entorno Ubuntu.

### Windows
#### Instrucciones para establecer el entorno de desarrollo en Windows.

1. Crear y activar virtual environment:

```
python -m venv /path_to_venv
cd /path_to_venv/Scripts
activate
```
2. Instalar requerimentos de software
```
pip install -r requirements_windows.txt
```

#### Con estas dependencias se puede modificar el diseño de la aplicación con python. 

### Linux

#### Instrucciones para establecer el entorno de desarrollo en Linux.

1. Crear y activar virtual environment:

```
python -m venv /path_to_venv
cd /path_to_venv
source bin/activate
```
2. Instalar requerimentos de software
```
pip install -r requirements_linux.txt
```

### Instalación de Java JDK 17 

#### A partir de este punto la instalación se realiza en Ubutnu 22.04. El Java JDK 17 es el lenguaje utilizado para usar la API de Android v. 31 y tener acceso a BLE.

#### Se abre una nueva terminal en Ubuntu 22.04
```
sudo apt update
sudo apt upgrade
sudo apt install openjdk-17-jdk
sudo update-alternatives --config java
sudo update-alternatives --config javac
```

### Instalación de Android SDK al entorno de desarrollo Linux
#### El Android SDK es necesario para poder accesder a la API que ofrece Android Developer para acceder a software y hardware especializado del celular. 

#### Instalar _command line tools_ marcado al inicio.
```
sudo apt install sdkmanager
mkdir -p ~/android-sdk/cmdline-tools
cd ~/android-sdk/cmdline-tools
unzip ~/Downloads/commandlinetools-linux-*.zip
mv cmdline-tools latest
cd
sdkmanager "platform-tools"
```
### Instalación de Android NDK al entorno de desarrollo Linux
#### Es necesario haber descargado Android NDK mostrado al inicio del documento

#### Se descomprime el archivo.
```
cd ~/Downloads
unzip android-ndk-version.zip -d ~/android-ndk
```
### Configuraciones adicionales para el entorno de desarrollo

#### Abrir .bashrc desde un editor de texto y agregar las siguientes líneas al final:

```
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH

export ANDROID_HOME=$HOME/android-sdk
export ANDROID_SDK_ROOT=$HOME/android-sdk
export PATH=$PATH:$ANDROID_HOME/cmdline-tools/latest/bin
export PATH=$PATH:$ANDROID_HOME/platform-tools
```
#### Cerrar el .bashrc_ y actualizaro

```
source ~/.bashrc
```
## Compilar apk usando buildozer

#### Al momento de clonar el repo, se agrega un archivo _buildozer.spec_ que describe las características de la compilación del apk. Si se desea generar uno desde cero:

```
cd ExoBoost/app
buildozer init
```

#### Para compilar la app se ejecuta:

```
cd ExoBoost/app
buildozer android clean
buildozer -v android debug
```
#### Es importante mencionar que _buildozer android clean_ solamente es necesario cuando se modifica el archivo _buildozer.spec_ o ocurre algún error durante la compilación.

## Android SDK
#### Cuando se compila la aplicación para generar el instalador apk se utiliza Python for Android (P4A), así como las herramientas de desarrollo de Android (Android SDK). La API de [Android v. 31](https://developer.android.com/reference/android/bluetooth/BluetoothGattCallback#onCharacteristicChanged(android.bluetooth.BluetoothGatt,%20android.bluetooth.BluetoothGattCharacteristic,%20byte[])) permite acceder a funciones tanto de software como en hardware, la cual está almacenada en clases de Java o Kotlin. Para esta aplicación se utilizaron las clases de Java. 
#### Bajo el directorio _javadev_, en el paquete _test_pkg_, se encuentran clases personalizadas de Java las cuales se importan a python utilizando ```pyjnius```.

## Debugger
#### Es necesario habilitar las opciones de desarrollador en el celular Android desde las configuraciones, así como el debugger por USB. Esto dependerá del fabricante y es necesario otorgar permisos especiales. 

#### Para debuggear la aplicación durante su desarrollo se utiliza la herramienta Android Debug Bridge (adb). 
#### Instalación: 
```
sudo apt install adb
```
#### Esta herramienta permite ver el logger de Android conecatado a la computadora. Para comprobar que esté correctamente instalado, conecta el celular por USB y ejecuta el siguiente comando:
```
adb devices
```
#### El dispositivo debería aparecer en la terminal. 
#### Durante la compilación, el celular se puede conectar y ejecutar este comando: 
```
adb logcat | grep python
```
#### Esto filtrará el logger por la palabra clave "python" y permitirá recibir mensajes para debuggera la aplicación. Adicionalmente, es posible compilar la aplicación e instalarla directamente por medio de adb: 
```
buildozer android debug deploy run
```

#### Debugging Ubuntu 22.04 en Virtual Box: 

Es común tener problemas al usar ADB debugger en VirtualBox mediante conexión USB, el error más común al momento de conectar un dispositivo Android de esta forma es ```Can't attach USB device``` y se puede resolver facilmente usando un filtro USB dentro de VirtualBox.

El origen del problema surge de un conflicto entre el uso del puerto USB entre la máquina host (Windows) y la máquina virtual (Ubuntu), ambas máquinas solicitan su uso al mismo tiempo y no se pueden intercambiar datos entre la PC y el dispositivo Android conectado.

Usualmente al momento de desplegar el error se desplegará el nombre, vendor ID y product ID, estos datos son los únicos importantes para crear un filtro que reconozca al dispositivo android solamente en la máquina virtual siempre que se conecte a la PC mediante USB.

Pasos para crear filtro:

1. Abrir VirtualBox Manager y seleccionar la máquina virtual (Ubuntu)
2. Ir al menú de configuración de la Virtual Machine (VM)
3. Abrir submenú USB
4. Seleccionar opción "Enable USB Controller"
5. Seleccionar la opción USB 3.0 (xHCI) Controller
6. Dentro del apartado "USB Device Filters" seleccionar el ícono con un signo de +
7. Llenar datos de Nombre, Vendor ID y Product ID (Ej: motorola moto g(9) plus, 22b8, 2e81)

Cabe mencionar que también es común que los campos Vendor ID y Product ID se llenen automáticamente al seleccionar el nombre del dispositivo en caso de estar conectado. Finalmente, se debe desconectar, iniciar la máquina virtual y volver a conectar el dispositivo después de iniciar completamente la máquina virtual.

Para más problemas de debugging se puede consultar la información dentro del siguiente [foro de VirtualBox](https://forums.virtualbox.org/viewtopic.php?f=35&t=82639)
***