'''Archivo para hacer pruebas de funcionamiento de pyjnius'''
from jnius import autoclass
Stack = autoclass('java.util.Stack')
stack = Stack()
stack.push("hello")
stack.push("world")

print(stack.pop())