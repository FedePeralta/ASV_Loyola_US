# Autonomous Surface Vehicles - Proyecto Universidad de Sevilla - Universidad Loyola

Códigos asociados a la implementación y despliegue de una flota de vehículos
autónomos de superficie para la monitorización de espacios ambientales.

### Roadmap actual:
[roadmap.md](roadmap.md)


### Uso
#### Configuration File:

Toda la configuración posible se encuentra en el archivo [ASV_DRONE_1.conf](ASV_DRONE_1.conf), en el cual se puede modificar cualquier parámetro antes de empezar el proceso.

Para empezar en modo RELEASE, se debe modificar el atributo DEBUG a FALSE, esto se puede hacer modificando el archivo:
```shell script
nano ASV_DRONE_1.conf
``` 

Reemplazando FALSE por TRUE en la línea 5 [ASV][DEBUG].

#### Modo Manual:
En una consola del raspberry, para el modo de despliegue:
```shell script
python3 main_as_service.py
```

#### Modo Automático:
[tutorial_service](tutorial_servicio.md)