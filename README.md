ApexIO - 23.09.2026

O projekcie

ApexIO to projekt przenośnego urządzenia do telemetrii pojazdu, pomiaru przyspieszenia i czasów okrążeń. Inspiracją były rozwiązania RaceBox i Racelogic, a celem było połączenie podobnych funkcji we własnym urządzeniu z ekranem i fizycznymi przyciskami. Projekt obejmuje oprogramowanie, własną płytkę PCB zaprojektowaną w KiCad oraz obudowę przygotowaną w Fusion 360.

Podstawą urządzenia jest ESP32-S3-WROOM. Dane o pozycji, prędkości i czasie dostarcza odbiornik GNSS NEO-F10N pracujący z częstotliwością 10 Hz, natomiast czujnik BNO085 mierzy przyspieszenie, prędkość kątową i pole magnetyczne. Wyniki są prezentowane na ekranie TFT o przekątnej 2,4 cala.

Oprogramowanie powstało w C++ z wykorzystaniem Arduino, PlatformIO i FreeRTOS. Sześć zadań rozdzielonych pomiędzy dwa rdzenie procesora odpowiada za odczyt czujników, przetwarzanie danych, obsługę przycisków i ekranu oraz zapis na kartę microSD. Zadania wymieniają dane przez kolejki, a dostęp do wspólnych zasobów jest zabezpieczony mutexami.

Do łączenia pomiarów GNSS i IMU wykorzystano bibliotekę z implementacją rozszerzonego filtra Kalmana (EKF). Dane z IMU pozwalają śledzić zmiany ruchu między odczytami GNSS, a pomiary satelitarne korygują narastające błędy oszacowania pozycji i prędkości. GeoLib przelicza współrzędne geograficzne na lokalny układ kartezjański, umożliwiając wykonywanie obliczeń odległości w metrach.



Funkcje

Live Telemetry

Tryb wyświetla aktualną prędkość, przeciążenia w osiach pojazdu, przebyty dystans, przybliżoną wysokość i kierunek ruchu. Prędkość jest wyznaczana na podstawie danych z filtra EKF i dodatkowo wygładzana filtrem medianowym. Zastosowano również ograniczenie wahań wskazania prędkości podczas postoju.

Drag Mode

Tryb mierzy czas przyspieszenia od 0 do 100 km/h oraz czas pokonania 60 stóp, 100 m, 200 m i 400 m. Wyświetla także prędkość maksymalną i aktualne przeciążenie. Po uzyskaniu gotowości pomiar rozpoczyna się automatycznie po wykryciu ruszenia, a po przejechaniu 400 m możliwy jest również pomiar drogi hamowania.

Lap Timer

Tryb mierzy czasy kolejnych okrążeń oraz pokazuje poprzedni i najlepszy wynik. Podczas okrążenia zapoznawczego tworzy wirtualne bramki, których kolejność przekraczania służy do sprawdzania poprawności kolejnych przejazdów. Funkcja live delta pokazuje bieżący zysk lub stratę czasu względem najlepszego prawidłowego okrążenia. Dane sesji są zapisywane na karcie microSD w formacie .vbo do późniejszej analizy w Circuit Tools 3.

Settings

Ustawienia pozwalają sprawdzić ustawienie urządzenia w osiach X i Y oraz wybrać przesunięcie czasu względem UTC. Można również zmieniać szerokość linii startu i mety oraz wirtualnych bramek. Dodatkowo dostępna jest regulacja zakresu paska pokazującego różnicę czasu okrążenia.



Wykorzystane biblioteki

- TFT_eSPI – obsługuje wyświetlacz TFT. Służy do rysowania ekranów, tekstu, grafik i wskaźników oraz wyświetlania wyników pomiarów.

- SparkFun u-blox GNSS v3 – odpowiada za komunikację z modułem GPS/GNSS. Umożliwia konfigurację odbiornika oraz odczyt pozycji, prędkości, czasu i liczby widocznych satelitów.

- SparkFun BNO08x Cortex Based IMU – obsługuje czujnik IMU. Dostarcza odczyty z akcelerometru, żyroskopu i magnetometru wykorzystywane do obliczania ruchu i orientacji urządzenia.

- Bolder Flight Systems Eigen – umożliwia wykonywanie obliczeń na macierzach i wektorach. W projekcie jest wykorzystywana w implementacji rozszerzonego filtra Kalmana, który łączy dane z czujników.

- SdFat – odpowiada za obsługę karty SD i plików. Umożliwia tworzenie plików z danymi przejazdu oraz zapisywanie kolejnych próbek telemetrii.

- Adafruit INA219 – obsługuje układ pomiaru napięcia i prądu. W projekcie służy do odczytu napięcia wykorzystywanego przez wskaźnik baterii.

- GeoLib - zawiera obliczenia związane ze współrzędnymi geograficznymi. Ułatwia przeliczanie pozycji na lokalny układ współrzędnych wykorzystywany w pomiarach przejazdu. 

- Adafruit BusIO – zapewnia funkcje komunikacji przez magistrale I²C i SPI. Jest biblioteką pomocniczą wykorzystywaną przez sterowniki Adafruit, między innymi INA219.

- Adafruit GFX Library – zawiera podstawowe funkcje rysowania tekstu i kształtów. Jest dołączona w zestawie bibliotek Adafruit; za interfejs wyświetlacza w ApexIO odpowiada TFT_eSPI.


