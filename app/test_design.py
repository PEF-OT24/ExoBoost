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
import platform
from kivy.clock import Clock
from kivymd.uix.label import MDLabel
Clock.max_iteration = 1000  # Increase this value if necessary

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class MainWindow(Screen):
    
    def search_devices(self):
        device_list = self.ids.device_list
        devices = [{'text': f'DISPOSITIVO {i}'} for i in range(1,4)]
        try:
            device_list.data = devices
        except:
            print('no devices aun')
    
    def connect_disconnect(self):
        device_list = self.ids.device_list
        selected_devices = [child for child in device_list.children[0].children if child.selected]
        try:
            if selected_devices:
                print(f"Connecting/Disconnecting {selected_devices[0].text}")
            else:
                print("No device selected")
        except:
            print("boton no funciona aun")

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
        return self.root
    
    def on_start(self):
        self.root.current = "Main Window"
    
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

    def bluetooth_connection(self): pass

    #------------------------ Métodos del menú de asistencia ------------------------

    def assitance_method(self): pass

    #------------------------ Métodos del menú de sintonizción ------------------------

    def limb_dropdown_clicked(self, limb: str) -> None: 
        self.limb = limb
        print(self.limb)

    def on_entry_text(self, value: str) -> None: 
        print(value)
    
def main():
    '''Initializes the app indicating the current OS'''
    TestDesignApp().run()

if __name__ == '__main__':
    main()
