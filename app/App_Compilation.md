# Compilación de App

## Windows
_Instrucciones para correr la aplicación en Windows_

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

### Requerimientos

_Instrucciones de como instalar los requerimientos en Linux haciendo un venv_

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

***