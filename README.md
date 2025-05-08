# Skylab Drone Prototype

## Description

Ce projet est un prototype de drone personnalisé développé par Skylab. Il s'appuie sur une architecture modulaire en C/C++ avec CMake et PlatformIO pour la compilation et le déploiement sur un microcontrôleur ESP32. L'objectif est de fournir une base flexible pour le développement de fonctionnalités de vol autonome, de télémétrie et de communication sans fil.

## Fonctionnalités

* Acquisition de données environnementales via le capteur BME280 (température, pression atmosphérique, humidité)
* Contrôle des moteurs brushless et gestion de la stabilisation avec MCPWM, Complementary filter et PID
* Communication série et sans fil (Wi-Fi/BLE/USB/ESP-NOW)
* Architecture logicielle modulable pour ajouter des capteurs et algorithmes de navigation

## Matériel requis

* Microcontrôleur ESP32 (ex. ESP32 DevKit)
* Capteur BME280 (température, pression, humidité)
* 4 × ESC (Electronic Speed Controller) pour moteurs brushless
* 4 × moteurs brushless
* Hélices adaptées
* Module d'alimentation LiPo (batterie)
* Câblage et breadboard ou PCB personnalisé
* Châssis du drone
* MPU9250 (IMU)

## Logiciels requis

* [PlatformIO](https://platformio.org) (extension pour VSCode ou CLI)
* Toolchain ARM pour ESP32 intégré à PlatformIO
* CMake 3.13 ou supérieur
* Python 3.x (pour les scripts de build/test)

## Installation

1. Clonez le dépôt :

   ```bash
   git clone https://github.com/SkylabYnov/skylab-drone.git --branch develop
   cd skylab-drone
   ```
2. Installez les dépendances :
   PlatformIO téléchargera automatiquement les bibliothèques requises
3. Compilez et téléversez le firmware :

   ```bash
   pio run --target upload
   ```

## Utilisation

## Structure du projet

```
├── include/          # Fichiers d'en-tête (.h)
├── lib/              # Bibliothèques tierces et modules
├── src/              # Code source principal (.c/.cpp)
├── test/             # Tests unitaires et d'intégration
├── CMakeLists.txt    # Configuration CMake pour ESP-IDF
├── platformio.ini    # Configuration PlatformIO
├── sdkconfig.esp32dev# Configuration ESP-IDF pour ESP32 DevKit
└── README.md         # Documentation du projet
```

## Contribution

Les contributions sont les bienvenues ! Merci de suivre ces étapes :

1. Fork du projet
2. Créez une branche pour votre fonctionnalité (`feature/ma-fonctionnalite`)
3. Faites vos modifications et tests
4. Ouvrez une Pull Request

## Licence

Ce projet est sous licence MIT. Voir le fichier [LICENSE](LICENSE) pour les détails.

## Contact

Pour toute question ou suggestion : [skylab@ynov.com](mailto:skylab@ynov.com)
