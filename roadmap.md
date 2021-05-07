# General Roadmap
* Configuration File
* Controlar alto nivel desde MQTT
* Controlar multiples vehiculos desde MQTT
* Generar como librería de python

## Specific Roadmap
* Crear archivo de configuración: el que va a ser leido de forma inicial por el main.py. Este debe contener toda la información parametrizada: como nombres de puertos, timeouts, nombres de topicos, numero de identificacion, verbose, logger data, etc.

* Volver a utilizar folders para mejorar presentación de la libreía:

      /Sensores
         /ArduinoSerialBoard.py
         /PumpModule.py
         /SensorModule.py
         /create_database.py
         /test_PumpModule.py
         /test_SensorModule.py
      
      /Utils
         /MQTT.py
         /resources_supervisor.py
         
      /kml_files
         /MisionesLoyola.kml
         /Misiones_Dron_US_Loyola_2.kml
         
* Conectar con MQTT

      Crear conexión de forma autónoma.