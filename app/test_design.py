# Set the screen size and mode
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Import kivy and kivymd libraries 
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from ColorManager import ColorManager
from kivy.clock import Clock
from kivymd.uix.label import MDLabel
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
Clock.max_iteration = 1000  # Increase this value if necessary

# Importar librerías para comunicación
import platform
import asyncio
import json 
import os
import math
from BLE import Connection, communication_manager

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class MainWindow(Screen): pass
class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class CustomLabelRoboto(MDLabel): pass # Case predefinida para los subtítulos con formato
class CustomLabelAD(MDLabel): pass # Case predefinida para los títulos con formato
class CustomTextEntry(MDTextField): pass # Case predefinida para las entradas de texto con formato

class TestDesignApp(MDApp):  
    #------------------------ Métodos de inicio ------------------------#
    def __init__(self, **kwargs):
        '''Initializes all methods, initial logical setup and define attributes'''
        super().__init__(**kwargs)
        self.kv_loaded: bool = False

        # Detecta el sistema operativo
        self.os_name = self.detect_os()
        self.pos_screen(0)

        # Diccionario de colores
        self.colors: dict = ColorManager()._get_colors()
        '''
        Available colors:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''

        self.limb: str = ""
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }
 
    def build(self):
        """Loads kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True
            # self.get_permissions() # Carga los permisos para Android, REVISAR 
        return self.root
    
    def on_start(self):
        self.root.current = "Main Window"

        self.device_list: GridLayout = self.root.get_screen("Main Window").ids.device_list

    async def launch_app(self):
        """Lazamiento de aplicación con el manejo de metodos asincronos"""
        await self.async_run(async_lib='asyncio')

    async def start(self):
        """Inicia la app de forma asincrona esperando que la tarea de lanzamiento finalize"""
        task = asyncio.create_task(self.launch_app())
        (_, pending) = await asyncio.wait({task}, return_when='FIRST_COMPLETED')
    
    # ------------------------ Administrador de ventanas ------------------------#
    def detect_os(self) -> str:
        '''Detects current OS of device and returns it as string'''
        os_name = platform.system()
        if os_name == 'Linux':
            return "Linux"
        elif os_name == 'Windows':
            return "Windows"
        elif os_name == 'Darwin':
            return "MacOS"
        elif os_name.startswith('Java'):
            if 'android' in platform.java_ver()[3][0]:
                return "Android"
        else:
            return "Unknown"

    def pos_screen(self, screen):
        '''Sets the screen position for each OS'''
        os_name = self.detect_os()
        if os_name == 'Linux':
            # Maximize the window on Linux
            Window.fullscreen = False
        elif os_name == 'Windows':
            # Fullscreen mode on Windows
            if screen == 0:
                Window.fullscreen = False
            else:
                Window.fullscreen = True
        elif os_name == 'Android':
            # Fullscreen mode on MacOS
            Window.fullscreen = True

    def on_slider_value(self, value):
        '''Handle the slider value change'''
        print(f"Slider value: {value}")

    def sit_down_stand_up(self):
        print("Sit down/stand up action triggered")

    def walk(self):
        print("Walk action triggered")

    def stop(self):
        print("Stop action triggered")

    #------------------------ Métodos de menú de blutooth ------------------------
    def search_devices(self):
        items = ["Item 1", "Item 2", "Item 3", "Item 4", "Item 5"]
        self.device_list.clear_widgets()  # Limpiar widgets anteriores
        self.displayed_items = []  # Resetear lista de elementos desplegados

        for item in items:
            btn = Button(text=item, size_hint_y=None, height=40)
            btn.bind(on_release=self.on_device_select)
            self.device_list.add_widget(btn)
            self.displayed_items.append(btn)
    
    def on_device_select(self, instance: str): print(f'{instance.text} fue presionado')

    def connect_disconnect(self): pass

    def get_permissions(self):
        """Solicita permisos de acceso a ubicación y bluetooth"""
        if self.os_name == 'android':
            from android.permissions import Permission, request_permissions  # type: ignore
            def callback(permission, results):
                if all([res for res in results]):
                    print('Got all permissions')
                else:
                    print('Did not get all permissions')
            try:
                request_permissions([Permission.BLUETOOTH,
                     Permission.BLUETOOTH_ADMIN, 
                     Permission.WAKE_LOCK, 
                     Permission.BLUETOOTH_CONNECT,
                     callback])
            except Exception as e:
                print(e)

    def start_BLE(self, flag: bool) -> None:
        """Metodo que inicia el proceso de conexión y comunicación BLE"""
        if flag:
            # ---- Hueco para activar el spinner al iniciar el bluetooth.
            # self.root.get_screen('main_window').ids.spinner.active = True
            try:
                self.ble_task = asyncio.create_task(run_BLE(self, self.dataTx_queue, self.battery_queue, self.deviceSelect_queue, self.angle_queue, self.manipulation_queue))
                self.update_battery_task = asyncio.ensure_future(self.update_battery_value())
                self.update_acceleration_task = asyncio.ensure_future(self.update_acceleration_value())
                self.update_speed_task = asyncio.ensure_future(self.update_speed_value())
                self.update_manipulation_task = asyncio.ensure_future(self.update_manipulation_value())
            except Exception as e:
                print(e)

    def device_clicked(self, _, value: str) -> None:
        """Metodo que inicializa el proceso de selección de dispositvo"""
        if value == "ESP32":
            self.device_clicked_task = asyncio.ensure_future(self.device_event_selected(value))

    async def device_event_selected(self, value: str) -> None:
        """Metodo que maneja el proceso de almacenar el dispostivo seleccionado en una queue y la transición a conexión"""
        await self.deviceSelect_queue.put(value)
        self.root.get_screen('main_window').ids.spinner.active = True
    #------------------------ Métodos del menú de asistencia ------------------------

    def assitance_method(self): pass

    #------------------------ Métodos del menú de sintonizción ------------------------

    def limb_dropdown_clicked(self, limb: str) -> None: 
        self.limb = limb
        print(self.limb)

    def on_entry_text(self, value: str) -> None: 
        print(value)

async def run_BLE(app: MDApp, dataTx_queue: asyncio.Queue, battery_queue: asyncio.Queue, deviceSelect_queue: asyncio.Queue,
                  angle_queue: asyncio.Queue, manipulation_queue: asyncio.Queue) -> None:
    """Método que inicia la conexión por el protocolo de BLE, asi como la comunicación entre servidor y cliente y el manejo de queues
       para el envio y repcion de datos"""
    
    read_char = "00002A3D-0000-1000-8000-00805f9b34fb"
    flag = asyncio.Event()
    connection = Connection(loop=loop,
                            uuid=UUID,
                            address=ADDRESS,
                            read_char=read_char,
                            write_char=read_char,
                            flag=flag,
                            app=app,
                            deviceSelect_queue=deviceSelect_queue)
    disconnect_flag['disconnect'] = False

    try:
        asyncio.ensure_future(connection.manager())
        asyncio.ensure_future(communication_manager(connection=connection,
                                                    write_char=read_char,
                                                    read_char=read_char,
                                                    dataTx_queue=dataTx_queue,
                                                    battery_queue=battery_queue, angle_queue=angle_queue,
                                                    manipulation_queue=manipulation_queue, disconnect_flag=disconnect_flag))
        print(f"fetching connection")
        await connection.flag.wait()
    finally:
        print(f"flag status confirmed!")

async def mainThread():
    """Hilo principal para el lanzamiento de la aplicación"""
    ExoBoostApp = TestDesignApp()
    task_runApp = asyncio.create_task(ExoBoostApp.start())
    (done, pending) = await asyncio.wait({task_runApp}, return_when='FIRST_COMPLETED')

if __name__ == '__main__':
    """Función principal que lanza la aplicación"""
    disconnect_flag = {'disconnect': False}
    ADDRESS, UUID = None, None
    loop = asyncio.get_event_loop()
    asyncio.run(mainThread())