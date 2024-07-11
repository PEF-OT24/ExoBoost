import os
from jnius import autoclass, JavaClass, java_method

# Establece el CLASSPATH al directorio que contiene el archivo .class
os.environ['CLASSPATH'] = 'com/example'

# Importa la clase Java
PythonScanCallback = autoclass('com.example.PythonScanCallback')

# Define una interfaz en Python para manejar los eventos
class PythonScanCallbackInterface(JavaClass):
    __javainterfaces__ = ['com.example.PythonScanCallback$Interface']

    @java_method('(I)V')
    def onScanFailed(self, errorCode):
        print(f"Scan failed with error code {errorCode}")

    @java_method('(ILjava/lang/String;)V')
    def onScanResult(self, callbackType, result):
        print(f"Scan result - Callback Type: {callbackType}, Result: {result}")

# Crea una instancia de PythonScanCallback y pásala a la clase Java
callback_instance = PythonScanCallback(PythonScanCallbackInterface())

# Prueba de llamadas
callback_instance.onScanFailed(1)
callback_instance.onScanResult(2, "Scan Result Data")
