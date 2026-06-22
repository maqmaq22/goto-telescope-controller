# GoTo Telescope Controller 🔭

Kompletny system sterowania montażem teleskopu GoTo oparty na Raspberry Pi i silnikach krokowych. Jest to zaawansowany projekt techniczny integrujący mechanikę, elektronikę oraz oprogramowanie do automatycznego śledzenia ciał niebieskich.

## 🌟 Główne funkcjonalności

* **Precyzyjne pozycjonowanie:** Wykorzystanie biblioteki Skyfield do obliczania pozycji obiektów astronomicznych (RA/DEC i ALT/AZ) w czasie rzeczywistym, w tym śledzenie Międzynarodowej Stacji Kosmicznej (ISS).
* **Computer Vision Tracking:** Moduł śledzenia wizyjnego przy użyciu kamery i biblioteki OpenCV, pozwalający na utrzymanie obiektów (np. Słońca, Księżyca, planet) w centrum kadru na podstawie analizy obrazu.
* **Kalibracja bez Polarnej:** Zaimplementowany autorski system kalibracji 3-punktowej, tworzący lokalny model sfery niebieskiej. Pozwala to na dokładne działanie systemu GoTo bez konieczności precyzyjnego ustawiania montażu na Gwiazdę Polarną.
* **Zabezpieczenia sprzętowe (Antykolizja):** System antykolizyjny oparty na czujnikach krańcowych (GPIO). W trybie automatycznym układ potrafi samodzielnie cofnąć oś i ominąć przeszkodę. *Uwaga: W systemie zaimplementowano również monitorowanie prądu silników za pomocą modułu INA219, jednak obecnie służy ono wyłącznie do telemetrii – pomiar prądowy nie wyzwala zatrzymania teleskopu przy kolizji.*
* **Webowy panel sterowania:** Asynchroniczny interfejs użytkownika zrealizowany w oparciu o Flask i SocketIO. Umożliwia ręczne sterowanie (joystick), wybór celów GoTo, podgląd z kamery na żywo oraz kalibrację.

## ⚙️ Wykorzystany sprzęt

* **Jednostka centralna:** Raspberry Pi
* **Napęd:** Silniki krokowe ze sterownikami HR8825
* **Zasilanie i telemetria:** Moduł UPS 3S z czujnikiem INA219 (I2C)
* **Czujniki:** Krańcówki (obsługa przez RPi.GPIO)
* **Wizja:** Kamera USB

## Autorzy
* Michał Makowski (@maqmaq22)
* Dawid Mańskowski
* Bartosz Majewski
* Mateusz Osełkowski


## 🚀 Uruchomienie projektu

1. Sklonuj repozytorium na Raspberry Pi:
   ```bash
   git clone [https://github.com/maqmaq22/goto-telescope-controller.git](https://github.com/maqmaq22/goto-telescope-controller.git)

   
