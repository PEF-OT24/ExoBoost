package com.example;

public class TestAndroidClass {
    // Atributo privado
    private String message;

    // Constructor
    public TestAndroidClass(String message) {
        this.message = message;
    }

    // Método para obtener el mensaje
    public String getMessage() {
        return message;
    }

    // Método para establecer el mensaje
    public void setMessage(String message) {
        this.message = message;
    }

    // Método para imprimir el mensaje
    public void printMessage() {
        System.out.println(message);
    }
}
