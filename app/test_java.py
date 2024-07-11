import os
from jnius import autoclass, PythonJavaClass, java_method

# Establecer el CLASSPATH al directorio que contiene los archivos .class
os.environ['CLASSPATH'] = '/com/example/'

# Importar la clase Java extendida
ExtendedClass = autoclass('com.example.ExtendedClass')

# Definir una clase en Python que hereda de la clase Java usando PythonJavaClass
class PythonExtendedClass(PythonJavaClass):
    __javaclass__ = 'com/example/ExtendedClass'

    def __init__(self, message):
        super().__init__(message)

    @java_method('()V')
    def additionalMethod(self):
        print("This is an additional method in the Python extended class.")

def main():
    # Crear una instancia de la clase extendida de Python
    python_obj = PythonExtendedClass('Hello from Python!')
    python_obj.printMessage()
    python_obj.additionalMethod()

if __name__ == '__main__':
    main()