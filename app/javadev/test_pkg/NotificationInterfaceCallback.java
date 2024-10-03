package javadev.test_pkg;

public interface NotificationInterfaceCallback {
    // Método que se ejecuta cada que se recibe una notificación.
    void processNotification(String serviceUUID, String characteristicUUID);
}