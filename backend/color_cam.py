import depthai as dai
import numpy as np
import cv2

if __name__ == "__main__":


    # Erstelle ein Pipeline-Objekt
    pipeline = dai.Pipeline()

    # Erstelle und konfiguriere die Kamera
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(30)

    # Erstelle ein Output-Node für das RGB-Bild
    xout = pipeline.createXLinkOut()
    xout.setStreamName("video")
    cam_rgb.video.link(xout.input)

    # Erstelle ein Gerät mit der Pipeline
    device = dai.Device(pipeline)

    # Holen der RGB-Bilder
    q_rgb = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    # Aufnahme von Bildern und Speichern
    while True:
        # Holen eines RGB-Bildes
        frame = q_rgb.get()

        # Konvertiere das Bild in ein NumPy-Array
        frame_rgb = frame.getCvFrame()

        # Zeige das Bild an
        cv2.imshow("RGB", frame_rgb)

        # Speichern des Bildes, wenn die 's'-Taste gedrückt wird
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite("captured_image.png", frame_rgb)
            print("Bild gespeichert!")

        # Beenden, wenn die 'q'-Taste gedrückt wird
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Schließe die Kamera und das Fenster
    device.close()
    cv2.destroyAllWindows()
