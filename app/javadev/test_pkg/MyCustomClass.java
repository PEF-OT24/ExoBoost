package javadev.src.test_pkg;

// import java.util.List;

public final class MyCustomClass {

    // Definición de la interfaz
    public interface Interface {
        void onScanFailed(int errorCode);  // Método de la interfaz para el fallo del escaneo
        void onScanResult(int callbackType, String result);  // Método de la interfaz para el resultado del escaneo
    }

    private Interface callback;

    public MyCustomClass(Interface pythonCallback) {
        this.callback = pythonCallback;
    }

    // Método para manejar el fallo del escaneo
    public void onScanFailed(int errorCode) {
        callback.onScanFailed(errorCode);
    }

    // Método para manejar el resultado del escaneo
    public void onScanResult(int callbackType, String result) {
        callback.onScanResult(callbackType, result);
    }
}
