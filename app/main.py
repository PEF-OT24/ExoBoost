# Se establece el tamaño y posición de la pantalla
from kivy.config import Config

# width = 400
# height = width*(9/20)
# Config.set('graphics', 'width', width)
# Config.set('graphics', 'height', height)
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Config.set('graphics', 'dpi', '387')

# Módulos internos
from ColorManager import ColorManager
from UUIDManager import UUIDManager

# Importar librerías de kivy
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout as Grid
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.popup import Popup

# Importar librerías de kivymd
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivymd.uix.label import MDLabel
from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton
from kivymd.uix.behaviors.toggle_behavior import MDToggleButton 

# GNZHT-CXNFD-H982D-WE7H3-29YL3

Clock.max_iteration = 1000  # Increase this value if necessary

# Importar librerías secundarias
from threading import Thread, Timer
import platform
import json 
import uuid
import webbrowser
from time import sleep

# Clase para mostrar el teclado en los text fields
from kivy.core.window import Window
Window.keyboard_anim_args = {'d': .2, 't': 'in_out_expo'}
Window.softinput_mode = 'pan'

class SplashScreen(Screen):
    '''Clase para mostrar la pantalla de inicio'''

    def on_enter(self, *args):
        '''Método que agenda el cambio'''
        Clock.schedule_once(self.switch_to_main,3)

    def switch_to_main(self, dt):
        '''Método que realiza el cambio a la pantalla principal'''
        self.manager.current = 'Main Window'

# Clase para el manejo de pantallas
class WindowManager(ScreenManager): pass # Administrador de pantallas
class MainWindow(Screen): pass           # Pantalla principal
class SecundaryWindow(Screen): pass      # Pantalla secundaria

# Clases predefinidas reciclables
# El formato específico y definido se encuentra en el archivo test.kv
class CustomLabelRoboto(MDLabel): pass        # Case predefinida para los subtítulos con formato
class CustomLabelAD(MDLabel): pass            # Case predefinida para los títulos con formato
class CustomTextEntry(MDTextField): pass      # Case predefinida para las entradas de texto con formato
class InfoPopUp(Popup): pass                  # Clase para mostrar un pop up de información
class ImageTeam(Image): pass                  # Clase para formatear las imágenes del equipo
class LabelTeam(MDLabel): pass                # Clase para formatear los datos del equipo
class ButtonDevices(MDFlatButton): pass       # Clase para crear botones de los dispositivos encontrados
class ButtonParameters(MDFlatButton, MDToggleButton): pass  # Clase para crear botones para seleccionar los parámetros a procesar

class ErrorPopup(Popup):
    '''Clase para mostrar un error genérico'''
    def __init__(self, **kwargs):
        super(ErrorPopup, self).__init__(**kwargs)

class ExoBoostApp(MDApp):
    #------------------------------------------------------- Métodos de inicio -----------------------------------------------------#
    def __init__(self, **kwargs):
        '''Se inicilizan todos los métodos, el set up de la lógica y se definen atributos'''
        super().__init__(**kwargs)

        # -------------------------- Atributos internos --------------------------
        self.kv_loaded: bool = False
        self.mode: str = None # Modo de operación la app: {assistance, tuning}
        
        # Detecta el sistema operativo
        self.os_name = self.detect_os()
        
        # Diccionario de etiquetas para la sintonización
        self.selected_limb: str = "Right leg"
        self.selected_param: str = "pos" # Valores posibles para el parámetro a elegir. ["pos", "vel", "cur"], modo de servo control 
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }
        
        # Límites de los parámetros PI de los motores
        self.motor_params_lims =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "99999"},
                "motor2": {"kc": "100", "ti": "50", "sp": "99999"},
                "motor3": {"kc": "100", "ti": "50", "sp": "99999"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "99999"},
                "motor2": {"kc": "50", "ti": "100", "sp": "99999"},
                "motor3": {"kc": "50", "ti": "100", "sp": "99999"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "99999"},
                "motor2": {"kc": "1", "ti": "1", "sp": "99999"},
                "motor3": {"kc": "1", "ti": "1", "sp": "99999"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "99999"},
                "motor2": {"kc": "10", "ti": "5", "sp": "99999"},
                "motor3": {"kc": "10", "ti": "5", "sp": "99999"},
            }
        }

        # -------------------------- Parámetros de BLE --------------------------
        if self.os_name == "Android" or self.os_name == "Linux":
            print("Importando librería de BLE")
            # Librería de BLE
            from BLEManager import BluetoothManager_App
            self.ble_found = True
        else: self.ble_found = False

        # Instancia de Bluetooth
        if self.ble_found: self.ble = BluetoothManager_App()
        else: self.ble = None

        # Atributos de lógica BLE
        self.selected_device: str = None           # Indica el nombre del dispositivo seleccionado
        self.connection_successful: bool = False   # Indica si la conexión fue exitosa
        self.reading: bool = False                 # Indica si la lectura de datos se encuentra activa

        # Se inicializa el hilo secundario de la lectura de datos
        self.read_thread = Thread(target=self.read_pv_cycle, args=(1000,))

        # -------- Manejo de los UUID según la ESP32 ---------
        self.uuid_manager = UUIDManager() # Ver UUIDManager.py
        names = ["Parameters", "Process", "Commands"] # Nombres de los servicios
        values = [0x0001, 0x0002, 0x0003]             # Sub UUIDs para los servicios
        self.uuid_manager.generate_uuids_services(names, values) # Se generan los UUIDs

        # Se generan las carcacterísticas para los servicios
        # --- Servicio de Parameters ---
        self.uuid_manager.generate_uuids_chars(names[0], ["PI", "SP","LEVEL"], [0x000a, 0x000f, 0x000d])
        # --- Servicio de Process ---
        self.uuid_manager.generate_uuids_chars(names[1], ["PV", "ALL_PV"], [0x000b, 0x000e])
        # --- Servicio de Commands ---
        self.uuid_manager.generate_uuids_chars(names[2], ["MODE"], [0x000c])

        # -------------------------- Atributos externos --------------------------
        # Diccionario de valores de los parámetros de los motores de sintonización y control
        
        # Diccionario de valores de los parámetros PI 
        self.motor_parameters_pi = {
            "limb": "Right leg", # "Right leg", "Left leg", "Right arm", "Left arm"
            "motor1": {
                "pos": {"kc": "100", "ti": "50"},
                "vel": {"kc": "100", "ti": "50"},
                "cur": {"kc": "100", "ti": "50"}
                },
            "motor2": {
                "pos": {"kc": "100", "ti": "50"},
                "vel": {"kc": "100", "ti": "50"},
                "cur": {"kc": "100", "ti": "50"}
                },
            "motor3": {
                "pos": {"kc": "100", "ti": "50"},
                "vel": {"kc": "100", "ti": "50"},
                "cur": {"kc": "100", "ti": "50"}
                }
        }

        # Diccionario de valores de los parámetros SP
        self.motor_setpoints = {
            "limb": "Right leg", 
            "monitoring": "pos", 
            "motor1": "0",
            "motor2": "0",
            "motor3": "0",
        }

        # Diccionario de valores de los parámetros PV
        self.motor_parameters_pv =  {
            "limb": "Right leg",
            "monitoring": "pos",
            "motor1": "0",
            "motor2": "0",
            "motor3": "0",
        }

        # Pantalla para desplegar app (número de monitor)
        self.pos_screen(0)

        # Diccionario de colores
        self.colors: dict = ColorManager()._get_colors() # Ver ColorManager.py

    def build(self):
        """Carga kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True

        # Diccionario de TextFields de sintonización para accceso rápido 
        self.param_pi_entries: dict[dict] = {
            "motor1": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor1,
                "ti": self.root.get_screen('Main Window').ids.ti_motor1,
                "sp": self.root.get_screen('Main Window').ids.sp_motor1
            },
            "motor2": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor2,
                "ti": self.root.get_screen('Main Window').ids.ti_motor2,
                "sp": self.root.get_screen('Main Window').ids.sp_motor2
            },
            "motor3": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor3,
                "ti": self.root.get_screen('Main Window').ids.ti_motor3,
                "sp": self.root.get_screen('Main Window').ids.sp_motor3
            },
        }

        # Diccionario de TextFields de variables de proceso para accceso rápido
        self.param_pv_entries: dict = {
            "motor1": self.root.get_screen('Main Window').ids.pv_motor1,
            "motor2": self.root.get_screen('Main Window').ids.pv_motor2,
            "motor3": self.root.get_screen('Main Window').ids.pv_motor3
        }

        # Se retorna la pantalla de inicio
        return self.root
    
    def on_start(self):
        self.root.current = "Splash Screen"

        # --------- Información de descripción del proyecto y equipo --------
        try:
            self.info_project = open("info_proyecto.txt", 'r', encoding='utf-8').read()
            self.team_info = [
                {"image": "images/TeresaHernandez.jpeg", "info": "Teresa Hernández\n\nIngeniería en Mecatrónica"},
                {"image": "images/CarlosReyes.jpeg", "info": "Carlos Reyes\n\nIngeniería en Mecatrónica"},
                {"image": "images/DavidVillanueva.jpeg", "info": "David Villanueva\n\nIngeniería en Mecatrónica"},
                {"image": "images/ItzelMartinez.jpeg", "info": "Itzel Martínez\n\nIngeniería en Mecatrónica & Biomédica"},
                {"image": "images/EduardoMartinez.jpeg", "info": "Eduardo Martínez\n\nIngeniería en Mecatrónica & Diseño Automotriz"}
            ]
        except: 
            # Información no encontrada
            print("Información no encontrada")
            self.team_info = self.info_project = "No info found"

        self.device_widgets_list: Grid = self.root.get_screen("Main Window").ids.device_list
        
        # Se configura como la extremidad inicial la pierna derecha
        self.limb_dropdown_clicked("Right leg")
    
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

    #----------------------------------------------------- Métodos generales ----------------------------------------------------
    def on_tab_select(self, tab: str): 
        '''Método que establece el modo de funcionamiento en función de la tab seleccionada'''
        if tab == "Assistance mode": 
            self.mode = "assistance"
            self.reading = True
        elif tab == "Bluetooth settings": 
            self.mode = "bluetooth"
            self.reading = False
        elif tab == "Monitoring tab": 
            self.mode = "monitoring"
            self.reading = True
        elif tab == "Tuning mode": 
            self.mode = "tuning"
            self.reading = True

    def is_valid(self, var: str, tipo) -> bool:
        """
        Valida si un dato en formato de string pertenece a otro tipo de dato. 
        Función para validación de informaicón.

        Entradas: var  str -> Dato a validar tipo string
                  tipo any -> valor correspondiente a la clase para validación deseada
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
        
    #----------------------------------------------------- Métodos de menú de Bluetooth -----------------------------------------
    def search_devices(self):
        '''Busca dispositivos Bluetooth, los almacena y los muestra en el ScrollView'''
        if self.ble_found:
            # Estado del bluetooth
            print(f"Bluetooth habilitado: {self.ble.is_bluetooth_enabled()}\n")  
            
            # Aqui se activa el spinner
            self.root.get_screen("Main Window").ids.loading_spinner.active = True
            
            # Lógica de escaneo
            # Se inicia el escaneo durante 5 segundos y se obtiene la lista de dispositivos
            self.ble.resetBLE() # Se reinica el escaneo
            self.ble.start_ble_scan()
            scanning = Thread(target=self.perfom_scanning, args=(5.0,))
            scanning.start()
        else: 
            print("Bluetooth no disponible")
            # Aqui el spinner se desactiva
            self.root.get_screen("Main Window").ids.loading_spinner.active = False
            devices = ["Dispositivo 1", "Dispositivo 2", "Dispositivo 3", "Dispositivo 4", "Dispositivo 5"] 
            # Se muestran los resutlados en la pantalla
            self.show_devices(devices)

    def show_devices(self, devices: list[str]):
        '''
        Método que muestra los elementos de una lista en el ScrollView
        Entrada: devices list[str] -> lista de dispositivos
        '''
        # Aqui el spinner tmb se desactiva
        self.root.get_screen("Main Window").ids.loading_spinner.active = False
        
        # --------- Lógica de la lista ---------------
        self.device_widgets_list.clear_widgets()  # Limpiar widgets anteriores
        print("show devices method")
        for dev in devices:
            btn = ButtonDevices(text=dev)
            btn.bind(on_release=self.on_device_select)
            self.device_widgets_list.add_widget(btn)
        
    def on_device_select(self, instance: ButtonDevices): 
        print("on device select method")
        '''Método que imprime dispositivo seleccionado'''
        self.selected_device = instance.text
        print(f'{instance.text} fue presionado')

        for widget in self.device_widgets_list.children:
            # Se cambia el color de fondo de los botones
            if widget.text == instance.text:
                widget.md_bg_color = self.colors["Light Orange"]
            else: 
                widget.md_bg_color = self.colors["Dark Blue"]

    def perfom_scanning(self, time: float = 5.0):
        self.is_scanning = True
        print("perfom scanning method")
        '''Método para iniciar escaneo de dispositivos''' 
        def stop_scanning(): 
            '''Detiene el escaneo y muestra los resultados'''
            self.root.get_screen("Main Window").ids.loading_spinner.active = False
            devices = self.ble.stop_ble_scan()
            # Se actualiza el ScrollView en el main Thread
            Clock.schedule_once(lambda x: self.show_devices(devices))
        # Acción que se ejecuta después de time segundos
        timer_ble = Timer(time, stop_scanning)
        timer_ble.start()

    def connect_disconnect(self): 
        '''Método para conectar/disconectar dispositivo'''

        def perform_connection():
            '''Método para realizar la conexión'''
            self.connection_successful = self.ble.connect(self.selected_device)
            Clock.schedule_once(update_label)

        def update_label(*args):
            self.root.get_screen('Main Window').ids.bt_state.text_color = self.colors["Green"]

        # No hace ninguna acción si no hay un dispositivo seleccionado o si el BLE no está disponible
        if not self.selected_device or not self.ble_found: return

        # Cuando no está conectado, se conecta
        if not self.ble.connected:
            t = Thread(target=perform_connection)
            t.start()
            print(f"Dispositivo conectado: {self.connection_successful}")
            self.root.get_screen('Main Window').ids.bluetooth_connect.text = "Disconnect"
            # Se cambia el texto del label y se muestra a que dispositivo se conectó
            self.root.get_screen('Main Window').ids.bt_state.text = f"Connected to {self.selected_device}"

            # Comienza la lectura de datos
            self.read_thread.start()

        # Cuando está conectado, se desconecta
        else:
            # Se realiza desconexión y se limpia el dispositivo seleccionado
            self.ble.disconnect()
            self.selected_device = None
            # Se cambia el texto del boton
            self.root.get_screen('Main Window').ids.bluetooth_connect.text = "Connect"
            # Se ambia el texto del label
            self.root.get_screen('Main Window').ids.bt_state.text = "Disconnected"
            self.root.get_screen('Main Window').ids.bt_state.text_color = self.colors["Red"]

    #----------------------------------------------------- Métodos del menú de asistencia -----------------------------------------------------
    # ---------------- Valor del slider ----------------
    def on_slider_value(self, value):
        '''Handle the slider value change'''
        print(f"Assitance Level: {value}")

        # Valida si se pueden enviar parámetros
        if not self.ble_found: return
        if not self.ble.connected: return

        # Se define la información a mandar con la limb
        json_data = {"assistance_level": str(value)}

        # Se definen los UUIDs y los datos a mandar para la parámetros de control 
        service_uuid = str(self.uuid_manager.uuids_services["Parameters"]) # Se convierte a string
        char_uuid = str(self.uuid_manager.uuids_chars["Parameters"]["LEVEL"]) # Se convierte a string

        # Se mandan los datos
        self.ble.write_json(service_uuid, char_uuid, json_data) 

    # ------- Botones de asistencia -------
    def sit_down_stand_up(self):
        '''Método para enviar el estado de sentarse/pararse'''
        
        # Acción de submit parámetros
        print("Acción de sentado/parado")
        if not self.ble_found: return

        # Se define la información a mandar con la limb
        json_data = {"state": "sit_down_stand_up"}
        json_data["limb"] = self.selected_limb

        # Se definen los UUIDs y los datos a mandar para la parámetros de control 
        service_uuid = str(self.uuid_manager.uuids_services["Commands"]) # Se convierte a string
        char_uuid = str(self.uuid_manager.uuids_chars["Commands"]["MODE"]) # Se convierte a string

        # Se mandan los datos
        if not self.ble.connected: return
        self.ble.write_json(service_uuid, char_uuid, json_data) 

    def walk(self):
        '''Método para enviar el estado de caminar'''
        
        # Acción de submit parámetros
        print("Acción de caminar")
        if not self.ble_found: return

        # Se define la información a mandar con la limb
        json_data = {"state": "walk"}
        json_data["limb"] = self.selected_limb

        # Se definen los UUIDs y los datos a mandar para la parámetros de control 
        service_uuid = str(self.uuid_manager.uuids_services["Commands"]) # Se convierte a string
        char_uuid = str(self.uuid_manager.uuids_chars["Commands"]["MODE"]) # Se convierte a string

        # Se mandan los datos
        if not self.ble.connected: return
        self.ble.write_json(service_uuid, char_uuid, json_data) 

    def stop(self):
        '''Método para enviar el estado de detenerse'''
        
        # Acción de submit parámetros
        print("Acción de detenerse")
        if not self.ble_found: return

        # Se define la información a mandar con la limb
        json_data = {"state": "stop"}
        json_data["limb"] = self.selected_limb

        # Se definen los UUIDs y los datos a mandar para la parámetros de control 
        service_uuid = str(self.uuid_manager.uuids_services["Commands"]) # Se convierte a string
        char_uuid = str(self.uuid_manager.uuids_chars["Commands"]["MODE"]) # Se convierte a string

        # Se mandan los datos
        if not self.ble.connected: return
        self.ble.write_json(service_uuid, char_uuid, json_data) 

    #----------------------------------------------------- Métodos del menú de sintonizción ----------------------------------------------------
    def load_params(self): 
        '''Método para cargar los parámetros guardados, se aplican para la extremidad seleccionada actualmente.'''
        print("load params method")

        with open('saved_parameters.json', 'r') as file:
            saved_params = json.load(file)
            print(saved_params)

        # Se guardan los parámetros
        self.motor_parameters_pi = saved_params
        self.motor_parameters_pi["limb"] = self.selected_limb

        # Se muestran los parámetros en la aplicación
        self.limb_dropdown_clicked(self.selected_limb)

    def save_params(self): 
        '''Método para guardar los parámetros escritos en un archivo .json
        Los parámetros se guardan de manera indistinta a la extremidad seleccionada.'''
        print("Save params method")

        # Se crea una copia para guardar sin el parámetro limb
        dict = self.motor_parameters_pi.copy()
        dict.pop("limb")
        
        # Se abre el archivo y se guardan los parámetros
        with open('saved_parameters.json', 'w') as file:
            json.dump(dict, file, indent=4)

    def limb_dropdown_clicked(self, limb: str) -> None: 
        '''
        Método para actualizar en la app los parámetros de los motores al seleccionar otra extremidad
        Entrada: Entrada seleccionada (str)
        '''

        # Se guarda la selección de la extremidad
        self.motor_parameters_pi["limb"] = self.selected_limb = limb
        self.motor_parameters_pv["limb"] = self.selected_limb = limb
        self.motor_setpoints["limb"]     = self.selected_limb = limb

        # Se cambian las etiquetas de los motores
        new_labels: list[str] = self.motors_labels[self.selected_limb]
        self.root.get_screen('Main Window').ids.motor1_label.text = new_labels[0]
        self.root.get_screen('Main Window').ids.motor2_label.text = new_labels[1]
        self.root.get_screen('Main Window').ids.motor3_label.text = new_labels[2]

        # MEJORA: SE DEBERÍA MANDAR UN REQUEST DE LECTURA PARA LEER LOS DATOS DEL MOTOR DEPENDIENDO DE LA EXTREMIDAD SELECCIONADA 
        # PARA MOSTRAR LA INFORMACIÓN ACTUALIZADA EN LA APP

        # Se cambian los valores de los parámetros PI de los motores en la aplicación
        # Se toman los valores previamente guardados
        # Motor 1
        self.root.get_screen('Main Window').ids.kc_motor1.text = self.motor_parameters_pi["motor1"][self.selected_param]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor1.text = self.motor_parameters_pi["motor1"][self.selected_param]["ti"]
        self.root.get_screen('Main Window').ids.pv_motor1.text = self.motor_parameters_pv["motor1"]
        # Motor 2
        self.root.get_screen('Main Window').ids.kc_motor2.text = self.motor_parameters_pi["motor2"][self.selected_param]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor2.text = self.motor_parameters_pi["motor2"][self.selected_param]["ti"]
        self.root.get_screen('Main Window').ids.pv_motor2.text = self.motor_parameters_pv["motor2"]
        # Motor 3
        self.root.get_screen('Main Window').ids.kc_motor3.text = self.motor_parameters_pi["motor3"][self.selected_param]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor3.text = self.motor_parameters_pi["motor3"][self.selected_param]["ti"]
        self.root.get_screen('Main Window').ids.pv_motor3.text = self.motor_parameters_pv["motor3"]

    def send_params(self): 
        '''Método para enviar parámetros PI al dispositivo conectado de todos los motores'''
        print("Método para enviar parámetros")
        print(self.motor_parameters_pi)
        if not self.ble_found: return

        # Se define la información a mandar
        json_data: dict = self.motor_parameters_pi

        # Selección de destino: se definen los UUIDs de la característica y el servicio 
        service_uuid = str(self.uuid_manager.uuids_services["Parameters"]) 
        char_uuid = str(self.uuid_manager.uuids_chars["Parameters"]["PI"]) 

        # Se mandan los datos hay conexión 
        if not self.ble.connected: return
        self.ble.write_json(service_uuid, char_uuid, json_data) 
    
    def send_sp(self): 
        '''Método para enviar parámetros SP al dispositivo conectado de todos los motores'''
        print("Método para enviar parámetros")
        print(self.motor_setpoints)
        if not self.ble_found: return

        # Se define la información a mandar
        json_data: dict = self.motor_setpoints

        # Selección de destino: se definen los UUIDs de la característica y el servicio 
        service_uuid = str(self.uuid_manager.uuids_services["Parameters"]) 
        char_uuid = str(self.uuid_manager.uuids_chars["Parameters"]["SP"]) 

        # Se mandan los datos hay conexión 
        if not self.ble.connected: return
        self.ble.write_json(service_uuid, char_uuid, json_data) 
    
    def process_var_select(self, instance, value) -> None:
        '''
        Método para establecer la variable de processo seleccionada.
        Dicha variable está ligada tanto al monitoreo de su valor como variable de proceso, 
        los valores de PI para la sintonización y su punto de set point para el control.
        '''
        if value == 'down': # Cuando un botón es presionado	
            print(f'Seleccionaste: {instance.text}')

            # Se establece el parámetro a sintonizar según la selección        
            match instance.text:
                case "Position":
                    self.selected_param = "pos"
                case "Velocity":
                    self.selected_param = "vel"
                case "Current":
                    self.selected_param = "cur"
            
            # Se guarda la variable seleccionada
            self.motor_setpoints["monitoring"] = self.selected_param
            self.motor_parameters_pv["monitoring"] = self.selected_param

            # Se cambian los valores de los parámetros PI de los motores en la aplicación
            # Se toman los valores previamente guardados
            # Motor 1
            self.root.get_screen('Main Window').ids.kc_motor1.text = self.motor_parameters_pi["motor1"][self.selected_param]["kc"]
            self.root.get_screen('Main Window').ids.ti_motor1.text = self.motor_parameters_pi["motor1"][self.selected_param]["ti"]
            self.root.get_screen('Main Window').ids.pv_motor1.text = self.motor_parameters_pv["motor1"]
            # Motor 2
            self.root.get_screen('Main Window').ids.kc_motor2.text = self.motor_parameters_pi["motor2"][self.selected_param]["kc"]
            self.root.get_screen('Main Window').ids.ti_motor2.text = self.motor_parameters_pi["motor2"][self.selected_param]["ti"]
            self.root.get_screen('Main Window').ids.pv_motor2.text = self.motor_parameters_pv["motor2"]
            # Motor 3
            self.root.get_screen('Main Window').ids.kc_motor3.text = self.motor_parameters_pi["motor3"][self.selected_param]["kc"]
            self.root.get_screen('Main Window').ids.ti_motor3.text = self.motor_parameters_pi["motor3"][self.selected_param]["ti"]
            self.root.get_screen('Main Window').ids.pv_motor3.text = self.motor_parameters_pv["motor3"]

    def on_entry_text(self, param: str, motor: str, value: str) -> None: 
        """
        Método que guarda el valor de entrada dependiendo si fue un parámetro o un set point. 
        Valida si el texto ingresado por el usuario es válido, tanto de clase como de rango.
        Entradas: param -> parámetro {'kc', 'ti', 'sp'}
                  motor -> número de motor {'motor1', 'motor2', 'motor3'}
                  value -> valor ingresado
        """
        if not self.is_valid(value, 1) and not self.is_valid(value, 1.0): # Valida si es int o float
            # Se pone el valor anteriormente guardado
            if param in ["kc", "ti"]:
                self.param_pi_entries[motor][param].text = self.motor_parameters_pi[motor][self.selected_param][param]
            else: 
                self.param_pi_entries[motor][param].text = "0"
            return

        if param in ["kc", "ti"]: # Parámetros de sintonización
            
            max_value = self.motor_params_lims[self.selected_limb][motor][param]
            if float(value) <= float(max_value) and float(value) >= 0.0: # Validación de valor máximo y posistivo
                # Si es válido, se actualiza el diccionario de parámetros
                self.motor_parameters_pi[motor][self.selected_param][param] = value
            else: # Valor no válido, reescribe el texto
                self.param_pi_entries[motor][param].text = self.motor_parameters_pi[motor][self.selected_param][param]
        else: # Parámetro de set point
            self.motor_setpoints[motor] = value

    def read_pv_cycle(self, time: int):  
        '''Método que leerá los datos de los motores perdiódicamente. Se ejecuta en un hilo separado.
        Este método se llama mientras se encuentre en modo sintonización.
        Entrada: time interval int -> Periodo de lectura de datos en ms'''

        sleep(2) # Espera un momento antes de comenzar la lectura
        while True:

            # Se valida que exista el dispositivo BLE, que esté conectado y lectura habilitada
            if not self.ble: continue
            if not self.ble.connected: continue
            if not self.reading: continue

            # Se realiza la lectura si está conectado y en lectura activa
            service_uuid = str(self.uuid_manager.uuids_services["Process"]) 
            char_uuid = str(self.uuid_manager.uuids_chars["Process"]["PV"]) 
            json_dict = self.ble.read_json(service_uuid, char_uuid) 

            ''' Estructura deseada del json
            json_dict = {
                "limb": "Rigth leg", " {"Rigth leg", "Left leg", "Right arm", "Left arm"}
                "monitoring": "pos", # {"pos", "vel", "cur"}
                "motor1": "100",
                "motor2": "100",
                "motor3": "100"
            }
            '''

            # Se obtienen los valores del diccionario
            try: 
                limb_read = json_dict["limb"]      
                monitoring_read = json_dict["monitoring"]      
                motor1pv_read = json_dict["motor1"]            
                motor2pv_read = json_dict["motor2"]            
                motor3pv_read = json_dict["motor3"]       
            except Exception as e:
                print("Error al leer los datos")
                print(e)  

            # Si es la variable de proceso de interés, se despliega la información 
            if not monitoring_read == self.motor_parameters_pv["monitoring"]: continue

            # Se guardan los valores en el diccionario
            self.motor_parameters_pv["motor1"] = motor1pv_read
            self.motor_parameters_pv["motor2"] = motor2pv_read
            self.motor_parameters_pv["motor3"] = motor3pv_read

            # Se muestran en pantalla los parámetros en la siguiente iteración de reloj
            if self.selected_limb == limb_read: Clock.schedule_once(self.update_process_variable)
            
            # Se espera el tiempo indicado para la siguiente lectura
            sleep(float(time/1000))

    def update_process_variable(self, *args):
        '''Método para actualizar la variable de proceso en la app'''
        self.root.get_screen('Main Window').ids.pv_motor1.text = self.motor_parameters_pv["motor1"]
        self.root.get_screen('Main Window').ids.pv_motor2.text = self.motor_parameters_pv["motor2"]
        self.root.get_screen('Main Window').ids.pv_motor3.text = self.motor_parameters_pv["motor3"]

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
        except: # En caso de error mostrar con una popup
            dialog = MDDialog(
                title="Error",
                text="Ha ocurrido un error.",
                buttons=[MDFlatButton(text="Cerrar", on_release=lambda *args: dialog.dismiss())]
            )
            dialog.open()

    def close_popup(self):
        '''Función para cerra el pop up de información'''
        self.popup.dismiss()

if __name__ == '__main__':
    """Función principal que lanza la aplicación"""
    ExoBoostApp().run()