# Se establece el tamaño y posición de la pantalla
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Módulos internos
from ColorManager import ColorManager

# Importar librerías
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivy.uix.popup import Popup
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from kivy.clock import Clock
from kivymd.uix.label import MDLabel
from kivy.uix.gridlayout import GridLayout as Grid
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton
from kivymd.uix.slider import MDSlider
from kivy.uix.dropdown import DropDown

Clock.max_iteration = 1000  # Increase this value if necessary

# Importar librerías para comunicación
import platform
import asyncio
import json 
import os
import math
import webbrowser
from BLE import Connection, communication_manager

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class SplashScreen(Screen):
    def on_enter(self, *args):
        Clock.schedule_once(self.switch_to_main,3)
    def switch_to_main(self, dt):
        self.manager.current = 'Main Window'

class MainWindow(Screen): pass
class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class CustomLabelRoboto(MDLabel): pass # Case predefinida para los subtítulos con formato
class CustomLabelAD(MDLabel): pass # Case predefinida para los títulos con formato
class CustomTextEntry(MDTextField): pass # Case predefinida para las entradas de texto con formato
class InfoPopUp(Popup): pass
class ImageTeam(Image): pass
class LabelTeam(MDLabel): pass
class ButtonDevices(MDFlatButton): pass
#class CustomScroll(GridLayout): pass

class TestDesignApp(MDApp):
    #------------------------ Métodos de inicio ------------------------#
    def __init__(self, **kwargs):
        '''Se inicilizan todos los métodos, el set up de la lógica y se definen atributos'''
        super().__init__(**kwargs)

        # -------------------------- Atributos internos --------------------------
        self.kv_loaded: bool = False
        self.mode: str = None # Modo de operación la app: {assistance, tuning}
        
        # Detecta el sistema operativo
        self.os_name = self.detect_os()
        
        # Diccionario de etiquetas para la sintonización
        self.limb: str = ""
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }
        
        # Límites de los parámetros PI de los motores
        self.motor_params_lims =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "0"},
                "motor2": {"kc": "100", "ti": "50", "sp": "0"},
                "motor3": {"kc": "100", "ti": "50", "sp": "0"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "0"},
                "motor2": {"kc": "50", "ti": "100", "sp": "0"},
                "motor3": {"kc": "50", "ti": "100", "sp": "0"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "0"},
                "motor2": {"kc": "1", "ti": "1", "sp": "0"},
                "motor3": {"kc": "1", "ti": "1", "sp": "0"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "0"},
                "motor2": {"kc": "10", "ti": "5", "sp": "0"},
                "motor3": {"kc": "10", "ti": "5", "sp": "0"},
            }
        }
        # -------------------------- Atributos externos --------------------------
        """
        Variables que se mandarán a través de bluetooth
        """
        # Diccionario de valores de los parámetros de los motores
        # Todos se inicializan con un valor arbitrario
        self.motor_parameters =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
                "motor2": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
                "motor3": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
                "motor2": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
                "motor3": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
                "motor2": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
                "motor3": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
                "motor2": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
                "motor3": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
            }
        }

        # Pantalla para desplegar app (no. de monitor)
        self.pos_screen(0)

        # Diccionario de colores
        self.colors: dict = ColorManager()._get_colors()
        '''
        Colores disponibles:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''
    def build(self):
        """Carga kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True

        # Diccionario de TextFields de sintonización para accceso rápido 
        self.param_entries: dict[dict] = {
            "motor1": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor1,
                "ti": self.root.get_screen('Main Window').ids.ti_motor1,
                "sp": self.root.get_screen('Main Window').ids.sp_motor1,
                "pv": self.root.get_screen('Main Window').ids.pv_motor1
            },
            "motor2": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor2,
                "ti": self.root.get_screen('Main Window').ids.ti_motor2,
                "sp": self.root.get_screen('Main Window').ids.sp_motor2,
                "pv": self.root.get_screen('Main Window').ids.pv_motor2
            },
            "motor3": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor3,
                "ti": self.root.get_screen('Main Window').ids.ti_motor3,
                "sp": self.root.get_screen('Main Window').ids.sp_motor3,
                "pv": self.root.get_screen('Main Window').ids.pv_motor3
            },
        }

        return self.root
    
    def on_start(self):
        self.root.current = "Splash Screen"

        # Se lee el archivo de texto incluyendo la información del proyecto
        try:
            self.info_project = open("info_proyecto.txt", 'r', encoding='utf-8').read()
        except: 
            # In case the file cannot be openned
            self.info_project = "No info found"
        
        # --------- Información de descripción del equipo --------
        self.team_info = [
            {"image": "images/TeresaHernandez.jpeg", "info": "Teresa Hernández\n\nIngeniería en Mecatrónica"},
            {"image": "images/CarlosReyes.jpeg", "info": "Carlos Reyes\n\nIngeniería en Mecatrónica"},
            {"image": "images/DavidVillanueva.jpeg", "info": "David Villanueva\n\nIngeniería en Mecatrónica"},
            {"image": "images/ItzelMartinez.jpeg", "info": "Itzel Martínez\n\nIngeniería en Mecatrónica & Biomédica"},
            {"image": "images/EduardoMartinez.jpeg", "info": "Eduardo Martínez\n\nIngeniería en Mecatrónica & Diseño Automotriz"}
        ]

        self.device_list: Grid = self.root.get_screen("Main Window").ids.device_list

    async def launch_app(self):
        """Lazamiento de aplicación con el manejo de metodos asincronos"""
        await self.async_run(async_lib='asyncio')

    async def start(self):
        """Inicia la app de forma asincrona esperando que la tarea de lanzamiento finalize"""
        task = asyncio.create_task(self.launch_app())
        (_, pending) = await asyncio.wait({task}, return_when='FIRST_COMPLETED')
    
    # ------------------------ Administrador de ventanas ------------------------#
    def detect_os(self) -> str:
        '''Detecta el OS y lo devuelve como string'''
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

    def pos_screen(self, screen: int):
        '''Establece modo pantalla completa dependiendo del OS'''
        os_name = self.detect_os()
        if os_name == 'Linux' or os_name == 'Windows':
            # Se maximiza/minimiza en windows de acuerdo al parámetro
            if screen == 0:
                Window.fullscreen = False
            else:
                Window.fullscreen = True
        elif os_name == 'Android':
            # Pantalla completa en Android
            Window.fullscreen = True

    #------------------------ Métodos generales ------------------------

    def on_tab_select(self, tab: str): 
        '''Método que establece el modo de funcionamiento en función de la tab seleccionada'''
        if tab == "Assistance mode": 
            self.mode = "assistance"
        elif tab == "Bluetooth settings": 
            self.mode = "bluetooth"
        elif tab == "Tuning mode": 
            self.mode = "tuning"
        
        print(self.mode)
    #------------------------ Métodos de menú de Bluetooth ------------------------

    def bluetooth_connection(self): pass

    def send_params(self): raise NotImplementedError("Not implemented function")

    #Crea lista de 5 dispositivos en el menú de bluetooth
    def search_devices(self):
        try:
            import BLE_python
        except:
            print("Library not imported propertly")
            self.display_devices()
    def display_devices(self):    
        items = ["Item 1", "Item 2", "Item 3", "Item 4", "Item 5"]
        self.device_list.clear_widgets()  # Limpiar widgets anteriores
        self.displayed_items = []  # Resetear lista de elementos desplegados

        for item in items:
            btn = ButtonDevices(text="                  "+item+"               ")
            btn.bind(on_release=self.on_device_select)
            self.device_list.add_widget(btn)
            self.displayed_items.append(btn)
    
    # Método que imprime dispositivo seleccionado
    def on_device_select(self, instance: str): print(f'{instance.text} fue presionado')

    def connect_disconnect(self): pass
    # Métodos para buscar dispositivos y conectarse 
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
    # ----------------------- Imprime valor del slider -----------------
    def on_slider_value(self, value):
        '''Handle the slider value change'''
        print(f"Assitance Level: {value}")
    #-------------------- Imprimen acciones en botones de asistencia -----------------
    # Pararse/Sentarse
    def sit_down_stand_up(self):
        print("Sit down/stand up action triggered")
    #Caminar
    def walk(self):
        print("Walk action triggered")
    #Detenerse
    def stop(self):
        print("Stop action triggered")

    def assitance_method(self): pass

    #------------------------ Métodos del menú de sintonizción ------------------------
    
    #Método para desplegar valores de PI en cada motor de acuerdo a la extremidad seleccionada
    def limb_dropdown_clicked(self, limb: str) -> None: 
        '''
        Función para actualizar los datos de los motores al seleccionar otra extremidad
        Entrada: Entrada seleccionada (str)
        '''

        # Se obtiene la selección
        self.limb = limb

        # Se cambian las etiquetas de los motores
        new_labels: list[str] = self.motors_labels[self.limb]
        self.root.get_screen('Main Window').ids.motor1_label.text = new_labels[0]
        self.root.get_screen('Main Window').ids.motor2_label.text = new_labels[1]
        self.root.get_screen('Main Window').ids.motor3_label.text = new_labels[2]

        # Se cambian los valores de los parámetros PI de los motores
        new_params: dict[dict[str]]= self.motor_parameters[self.limb]
        # Motor 1
        self.root.get_screen('Main Window').ids.kc_motor1.text = new_params["motor1"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor1.text = new_params["motor1"]["ti"]
        # Motor 2
        self.root.get_screen('Main Window').ids.kc_motor2.text = new_params["motor2"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor2.text = new_params["motor2"]["ti"]
        # Motor 3
        self.root.get_screen('Main Window').ids.kc_motor3.text = new_params["motor3"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor3.text = new_params["motor3"]["ti"]

    def on_entry_text(self, param: str, motor: str, value: str) -> None: 
        """
        Método que valida si el texto ingresado por el usuario es válido, tanto de clase como de rango.
        Entradas: param -> parámetro {'kc', 'ti', 'sp'}
                  motor -> número de motor {'motor1', 'motor2', 'motor3'}
                  value -> valor ingresado
        """
        old_params: dict[dict[str]] = self.motor_parameters[self.limb]

        if self.is_valid(value, 1): # Validación de dato como int
            if param in ["kc", "ti", "sp"]:
                max_value = self.motor_params_lims[self.limb][motor][param]
                if int(value) <= int(max_value) and int(value) >= 0: # Validación de rango válido
                    # Si es válido, se actualiza el diccionario de parámetros
                    self.motor_parameters[self.limb][motor][param] = value
                else: # Valor no válido
                    self.param_entries[motor][param].text = old_params[motor][param]
        else: # Tipo no válido
            self.param_entries[motor][param].text = old_params[motor][param]
    
    # --------------------------- Métodos del menú Pop Up -------------------------
    def show_popup(self):
        self.popup = InfoPopUp()
        self.popup.open()
        
        # Se agrega la información de cada miembro al layout
        grid: Grid = self.popup.ids.team_grid
        for person in self.team_info: 
            grid.add_widget(ImageTeam(source = person["image"]))
            grid.add_widget(LabelTeam(text = person["info"]))

    def open_repo(self) -> None: 
        '''
        Función para abrir el repositorio del código fuente
        
        Nota: el usuario necesita acceso al repo para poder abrirlo
        '''
        url: str = "https://github.com/PEF-OT24/ExoBoost"
        try: 
            webbrowser.open(url)
        except:
            dialog = MDDialog(
            title="Error",
            text="Ha ocurrido un error.",
            buttons=[
                MDFlatButton(
                    text="Cerrar", on_release=lambda *args: dialog.dismiss()
                )
            ],
            )
            dialog.open()

    def close_popup(self):
        '''Función para cerra el pop up de información'''
        self.popup.dismiss()

    # --------------------------- Funciones de uso general -------------------------

    def is_valid(self, var: str, tipo) -> bool:
        """
        Valida si un dato en formato de string pertenece a otro tipo de dato. 
        Función para validación de informaicón

        Entradas: var  - Dato a validar tipo string
                  tipo - valor correspondiente a la clase para validación
        Salida: Booleano indicando si el dato pertenece a la clase indicada
        """
        try:
            if isinstance(tipo, int):
                int(var)
            elif isinstance(tipo, float):
                float(var)
            else:
                raise TypeError("La clase de 'tipo' no está considerada")
            return True
        except (ValueError, TypeError):
            return False

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
