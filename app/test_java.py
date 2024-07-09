from jnius import autoclass

# Load the custom Java class
MyCustomClass = autoclass('com.example.MyCustomClass')

# Create an instance of the Java class
my_instance = MyCustomClass('Hello from Java!')

# Call methods on the Java object
print(my_instance.getMessage())  # Output: Hello from Java!
my_instance.setMessage('Hello from Python!')
my_instance.printMessage()  # This will print: Hello from Python!