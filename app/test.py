import asyncio
import threading
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock

class MyApp(App):
    def build(self):
        self.layout = BoxLayout()
        self.label = Label(text="Esperando actualización...")
        self.layout.add_widget(self.label)

        # Iniciar el bucle de eventos de asyncio en un hilo separado
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.start_loop, args=(self.loop,))
        self.thread.start()

        # Iniciar la tarea asíncrona
        asyncio.run_coroutine_threadsafe(self.ciclo_asincrono(), self.loop)

        return self.layout

    def start_loop(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()

    async def ciclo_asincrono(self):
        contador = 0
        while True:
            contador += 1
            # Utilizar Clock.schedule_once para interactuar con la GUI
            Clock.schedule_once(lambda dt: self.actualizar_etiqueta(contador), 0)
            await asyncio.sleep(1)  # Esperar 1 segundo

    def actualizar_etiqueta(self, valor):
        self.label.text = f"Contador: {valor}"

    def on_stop(self):
        # Terminar el bucle de eventos de asyncio cuando la aplicación se cierre
        self.loop.stop()
        self.thread.join()

if __name__ == "__main__":
    MyApp().run()
