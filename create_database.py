#importar librer1a de sqlite
import sqlite3

#nos conectamos con la base de datos llamada prueba1.db
#si no existe crea una con ese nombre en el mismo directorio
conectar=sqlite3.connect("LOCAL_DATABASE.db")

#Creamos un cursor que nos permitira interactuar con la base de datos
cursor=conectar.cursor()

#ahora creamos la tabla con los datos que guardaremos
cursor.execute("""CREATE TABLE sensor(
                ID text,
		SAMPLE_NUM real,
                BAT real,
                TEMP real,
                PH real,
                DO real,
                LATITUD real,
                LONGITUD real,
                COND real,
                ORP real,
                DATE text
                )""")

#el siguiente comando "commit()" guarda la tabla creada
conectar.commit()

#cerramos la conexion con la base de datos
conectar.close()
