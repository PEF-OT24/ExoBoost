package javadev.test_pkg;

public class ExtendedClass extends TestAndroidClass {

    public ExtendedClass(String message) {
        super(message);
    }

    // Método adicional en la clase extendida
    public void additionalMethod() {
        System.out.println("This is an additional method in the extended class.");
    }
}