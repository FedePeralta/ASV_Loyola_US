# Tutorial para crear y ejecutar el programa principal como servicio

Optimizado para versión de prueba (no como librería de python)

* Paso 1: Crear servicio:
```shell script
sudo nano /etc/systemd/system/asv_service.service 
```
* Paso 2: Llenar el documento (verificar si el WorkingDirectory coincide):

        [Unit]
        Description=ASV Loyola-US dronekit controller Service
        
        [Service]
        User=ubuntu
        WorkingDirectory=/home/ubuntu/ASV_Loyola_US/
        ExecStart=/home/ubuntu/ASV_Loyola_US/script
        TimeoutStopSec=10
        Restart=on-failure
        RestartSec=5
        
        [Install]
        WantedBy=multi-user.target
* Paso 3: Dar permiso al script
```shell script
sudo chmod u+x script
```

* Paso 4: Iniciar el servicio
```shell script
sudo systemctl daemon-reload
sudo systemctl enable asv_service.service
sudo systemctl start asv_service
sudo systemctl status asv_service
```

* Para detener/deshabilitar el servicio:
```shell script
sudo systemctl stop asv_service
sudo systemctl disable asv_service
```