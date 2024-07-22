import os
from jnius import autoclass, PythonJavaClass, java_method, JavaClass

os.environ['CLASSPATH'] = 'javadev'

# Importar la clase Java extendida
ExtendedClass = autoclass('javadev.test_pkg.ExtendedClass')

# Definir una clase en Python que hereda de la clase Java usando PythonJavaClass
class PythonExtendedClass(JavaClass):
    __javaclass__ = 'javadev/test_pkg/ExtendedClass'

    def __init__(self, message):
        print("Initializing PythonExtendedClass")
        self.instance = ExtendedClass(message)
    
    # Métodos de la clase original
    @java_method('()Ljava/lang/String;')
    def getMessage(self):
        print("get message")
        return self.instance.getMessage()

    @java_method('(Ljava/lang/String;)V')
    def setMessage(self, message):
        print("set message")
        self.message = self.instance.setMessage(message)

    @java_method('()V')
    def printMessage(self):
        print("print message")
        # self.instance.printMessage()

    # Métodos nuevos de la clase extendida
    @java_method('()V')
    def additionalMethod(self):
        print("Additional method")
        self.instance.additionalMethod()

def main():
    # Crear una instancia de la clase extendida de Python
    print("Ejecutando programa...")

    # Ejemplo simple para crear clases de java a Python
    python_obj = PythonExtendedClass('Hello from Python!')
    python_obj.printMessage()
    python_obj.additionalMethod()

if __name__ == '__main__':
    main()