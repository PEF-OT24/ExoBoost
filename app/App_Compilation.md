# Compilación de App

## Windows
_Instrucciones para ejecutar la aplicación en Windows_

1. Descargar el archivo requirements_windows.txt
2. Crear un virtual environment usando el siguiente comando:

```
python -m venv /path/to/new/virtual/environment
```
2. continuacion se debe activar para instalar los requerimientos necesarios_

```
source activate
```
5. Instalar los requerimientos
```
pip install -r requirements_linux.txt
```


## Linux

### Setup

_Instrucciones para instalación de requerimientos en Linux haciendo un venv_

1. Descargar archivo requirements_linux.txt
2. Crear una carpeta dentro de Documents destinada al environment
3. Seguir los siguientes pasos en orden para crear el environment
```
mkdir directory_env
sudo apt install python3-venv
python3 -m venv venvname
cd /bin
```
4. Activar el environment creado

```
source activate
```
5. Instalar los requerimientos
```
pip install -r requirements_linux.txt
```

### Compilación de apk usando Buildozer

_Instrucciones para generar el apk dentro de Ubuntu usando Buildozer_

_Aqui podemos poner que vayan a leer el archivo de steps_linux.txt o hacer otro readme lol nose o hacer un video explicativo como los de ebike_

_pondré las instrucciones aqui por mientras_

##### Instalación de Java 17 en Linux

```
sudo apt update
sudo apt install openjdk-17-jdk
sudo update-alternatives --config java
sudo update-alternatives --config javac
```
#### Instalación de Android SKD al entorno de desarrollo Linux

_Descargar las herramientas necesarias desde el [Sitio Oficial de Android](https://developer.android.com/studio?hl=es-419#downloads)_

_Seguir las siguientes lineas de código para instalar las herramientas descargadas_

```
sudo apt install sdkmanager
mkdir -p ~/android-sdk/cmdline-tools
cd ~/android-sdk/cmdline-tools
unzip ~/Downloads/commandlinetools-linux-*.zip
mv cmdline-tools latest
cd
sdkmanager "platform-tools"
```
#### Instalación de Android NDK al entorno de desarrollo Linux

_Descargar las herramientas necesarias desde el [Sitio Oficial de Android](https://developer.android.com/ndk/downloads?hl=es-419)_

_Seguir las siguientes líneas de código para instalar las herramientas descargadas_

```
cd ~/Downloads
unzip android-ndk-version.zip -d ~/android-ndk
```
#### Configuraciones adicionales para el entorno de desarrollo

_Abrir .bashrc desde un editor de texto y agregar las siguientes líneas_

```
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH

export ANDROID_HOME=$HOME/android-sdk
export ANDROID_SDK_ROOT=$HOME/android-sdk
export PATH=$PATH:$ANDROID_HOME/cmdline-tools/latest/bin
export PATH=$PATH:$ANDROID_HOME/platform-tools
```
_Cerrar el .bashrc_

```
source ~/.bashrc
```
#### Compilar apk usando buildozer

_Inicializar Buildozer previamente instalado en el virtual environment_

```
cd /path/to/your/project
buildozer init
```
_Posteriormente se debe editar el archivo .spec de Buildozer a preferencia del usuario, accediendo al mismo mediante el shell de Linux o bien, usando un editor de texto. A continuación se deben seguir los siguientes pasos para compilar el apk:_

```
buildozer android clean
buildozer -v android debug
```
_Cabe mencionar que esto se debe realizar en la misma ruta donde se encuentran los archivos de la aplicación móvil y con el virtual environment activado_




***