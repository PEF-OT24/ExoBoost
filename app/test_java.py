from jnius import autoclass, PythonJavaClass, java_method

# Implementación de una clase de Java en Python, definida a partir de un archivo .class

# Se impoarta la clase
MyCustomClass = autoclass('com.example.MyCustomClass')

class TestJava(PythonJavaClass):
    __javaclass__ = 'com.example.MyCustomClass'  # Se indica la clase

    def __init__(self, message):
        super().__init__()
        self.messages = []
        
    @java_method('()Ljava/lang/String;')
    def getMessage(self):
        pass  

    @java_method('(Ljava/lang/String;)V')
    def setMessage(self, message):
        '''Añadir nuevos mensajes'''
        self.messages.append(message)

    @java_method('()V')
    def printMessage(self):
        pass  

    # Additional Python methods
    def returnMessages(self):
        return self.messages

if __name__ == "__main__":
    # Create an instance of MyCustomClassWrapper
    testjavaclass = TestJava("Hello from Python!")

    # Call Java methods through the wrapper
    print("Original message:", testjavaclass.getMessage())
    testjavaclass.setMessage("Adding more text")
    print("Updated message:", testjavaclass.getMessage())

    # Call the Java printMessage method
    print("Último mensaje")
    testjavaclass.printMessage()

    print("Todos los mensajes")
    print(testjavaclass.returnMessages())