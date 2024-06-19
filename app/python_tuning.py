from kivy.lang import Builder
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.app import App
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField

class TuningModeScreen(Screen):
    pass

class MainApp(MDApp):
    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "BlueGray"
        Builder.load_file("tuning.kv")  # Replace with your actual kv file name
        sm = ScreenManager()
        sm.add_widget(TuningModeScreen(name="tuning_mode"))
        return sm

if __name__ == "__main__":
    MainApp().run()
