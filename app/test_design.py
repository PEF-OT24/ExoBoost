# Set the screen size and mode
from kivy.config import Config
Config.set('graphics', 'width', '540')
Config.set('graphics', 'height', '926')
Config.set('graphics', 'fullscreen', '0')

# Import kivy and kivymd libraries 
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from ColorManager import ColorManager
import platform
from kivy.clock import Clock
from kivymd.uix.label import MDLabel
Clock.max_iteration = 1000  # Increase this value if necessary

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain different or new functions to the app
class MainWindow(Screen): pass 
class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class CustomLabelAD(MDLabel): pass

class TestDesignApp(MDApp):
    def __init__(self, **kwargs):
        '''Initializes all methods, initial logical setup and define attributes'''
        super().__init__(**kwargs)
        self.kv_loaded: bool = False

        # Detects OS running
        self.os_name = self.detect_os()
        self.pos_screen(0)

        # Colors dictionary used on the design file
        self.colors: dict = ColorManager()._get_colors()
        '''
        Available colors:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''

    def build(self):
        """Loads kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True
        return self.root
    
    def on_start(self):
        self.root.current = "Main Window"
        # Bind the slider's value to a function
        #self.root.ids.assistance_level_slider.bind(value=self.on_slider_value_change)

    def detect_os(self) -> str:
        '''Detects current OS of device and returns it as string'''
        os_name = platform.system()
        if os_name == 'Linux':
            return "Linux"
        elif os_name == 'Windows':
            return "Windows"
        elif os_name == 'Darwin':
            return "MacOS"
        else:
            return "Unknown"

    def pos_screen(self, screen):
        '''Sets the screen position for each OS'''
        os_name = self.detect_os()
        if os_name == 'Linux':
            # Maximize the window on Linux
            Window.maximize()
        elif os_name == 'Windows':
            # Fullscreen mode on Windows
            if screen == 0:
                Window.fullscreen = False
            else:
                Window.fullscreen = True
        elif os_name == 'MacOS':
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

def main():
    '''Initializes the app indicating the current OS'''
    TestDesignApp().run()

if __name__ == '__main__':
    main()
