
# GoTo Telescope Controller 🔭

Kompletny system sterowania montażem teleskopu GoTo oparty na Raspberry Pi i silnikach krokowych. Projekt zrealizowany w ramach pracy inżynierskiej, integrujący mechanikę, elektronikę oraz oprogramowanie do automatycznego śledzenia ciał niebieskich.

## 🌟 Główne funkcjonalności

* *Precyzyjne pozycjonowanie:* Wykorzystanie biblioteki Skyfield do obliczania pozycji obiektów astronomicznych (RA/DEC i ALT/AZ) w czasie rzeczywistym, w tym śledzenie Międzynarodowej Stacji Kosmicznej (ISS).
* *Computer Vision Tracking:* Moduł śledzenia wizyjnego przy użyciu kamery i biblioteki OpenCV, pozwalający na utrzymanie obiektów (np. Słońca, Księżyca, planet) w centrum kadru na podstawie analizy obrazu.
* *Kalibracja bez Polarnej:* Zaimplementowany autorski system kalibracji 3-punktowej, tworzący lokalny model sfery niebieskiej. Pozwala to na dokładne działanie systemu GoTo bez konieczności precyzyjnego ustawiania montażu na Gwiazdę Polarną.
* *Zabezpieczenia sprzętowe:* Podwójny system antykolizyjny – sprzętowy (czujniki krańcowe na GPIO) oraz prądowy (monitorowanie przeciążeń silników za pomocą modułu INA219). W trybie automatycznym system potrafi samodzielnie cofnąć oś i ominąć przeszkodę.
* *Webowy panel sterowania:* Asynchroniczny interfejs użytkownika zrealizowany w oparciu o Flask i SocketIO. Umożliwia ręczne sterowanie (joystick), wybór celów GoTo, podgląd z kamery na żywo oraz kalibrację.

## ⚙️ Wykorzystany sprzęt

* *Jednostka centralna:* Raspberry Pi
* *Napęd:* Silniki krokowe ze sterownikami HR8825
* *Zasilanie i telemetria:* Moduł UPS 3S z czujnikiem INA219 (I2C)
* *Czujniki:* Krańcówki (obsługa przez RPi.GPIO)
* *Wizja:* Kamera USB

## 🚀 Uruchomienie projektu

1. Sklonuj repozytorium na Raspberry Pi:
   ```bash
   git clone [https://github.com/maqmaq22/goto-telescope-controller.git](https://github.com/maqmaq22/goto-telescope-controller.git)
