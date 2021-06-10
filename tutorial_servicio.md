{::options parse_block_html="true" /}

# Tutorial para crear y ejecutar el programa principal como servicio

Optimizado para versión de prueba (no como librería de python)

* Paso 1: Crear servicio:
```shell script
sudo nano /etc/systemd/system/asv_service.service 
```
* Paso 2: Llenar el documento :

Verificar si User y WorkingDirectory coinciden!
{: .alert .alert-danger}

        [Unit]
        Description=ASV Loyola-US dronekit controller Service
        Wants=network-online.target
        After=network-online.target
        
        [Service]
        User=pi
        WorkingDirectory=/home/pi/ASV_Loyola_US/
        ExecStart=/urs/bin/python3 main_as_service.py &
        TimeoutStopSec=10
        Restart=on-failure
        RestartSec=5
        
        [Install]
        WantedBy=multi-user.target
 Si es un entorno grafico, puede pegar el documento clickeando en editar y luego pegar.

* Paso 3: Iniciar el servicio:
```shell script
sudo systemctl daemon-reload
sudo systemctl enable asv_service.service
sudo systemctl start asv_service
sudo systemctl status asv_service
```

* Para ver todo el log, ejecutar:
```shell script
journalctl -u asv_service.service 
```

* Para detener/deshabilitar el servicio:
```shell script
sudo systemctl stop asv_service
sudo systemctl disable asv_service
```