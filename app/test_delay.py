import threading
import time

class CLASE:
    def __init__(self):
        self.evento = threading.Event()
        self.resultado = None

    def escanear(self):
        # Simular una operación de escaneo que tarda un tiempo
        self.resultado = "dispositivos encontrados"
        # Notificar al hilo principal que el escaneo ha terminado
        self.evento.set()

    def encontrar_dispositivos(self):
        # Crear e iniciar un hilo para la función escanear
        hilo_escanear = threading.Timer(5, self.escanear)
        hilo_escanear.start()

        # Esperar a que el escaneo termine sin bloquear el hilo principal
        print("Esperando los resultados del escaneo...")
        self.evento.wait()
        
        # Continuar con el resto del código después de que el escaneo haya terminado
        print(f'Resultados del escaneo: {self.resultado}')
        # Aquí puedes continuar con el resto de tu lógica que depende de los resultados
    
    def metodo(self):
        t = threading.Thread(target=self.encontrar_dispositivos)
        t.start()

# Crear una instancia de la clase
objeto = CLASE()

# Llamar al método encontrar_dispositivos
objeto.metodo()

print("Programa terminado")
