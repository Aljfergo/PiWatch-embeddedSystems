from fastapi import FastAPI, HTTPException
import psycopg2
from config import config
from pydantic import BaseModel
from colorama import Style, Fore, Back, init
from fastapi.middleware.cors import CORSMiddleware
from datetime import datetime, timedelta

app=FastAPI()



#
#   Configuración de las políticas de CORS (todo permitido)
#

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_methods=["*"],  
    allow_headers=["*"], 
    allow_credentials=True, 
    expose_headers=["*"],  
)



#
#   Base models para los bodies
#

class UserCredentials(BaseModel):
    username: str
    password: str
    token: str


class Schedule(BaseModel):
    scheduleStart: str
    scheduleEnd: str

class Incident(BaseModel):
    #timestamp: str
    incidentpic: str
    severity: int




#
#   Conexión inicial
#

def connect():
    conn=None

    print("Conecta o no conecta pero hay que saberlo")
    
    try:
        params=config()
        init()
        print(Fore.YELLOW+"->Conectando con la base de Datos...\n\n")
        conn =psycopg2.connect(**params)

        cur=conn.cursor()
        print(Style.RESET_ALL+'* Postgre version ')
        cur.execute('SELECT version()')
        db_version=cur.fetchone()
        print(db_version)
        print('\n')
        print(Fore.GREEN+"->Base de datos conectada correctamente\n\n")

        with open("bdd/piwatch_structure.sql", "r") as script_file:
            cur.execute(script_file.read())
            
            print("Tablas creadas con éxito")
        
        cur.close()
        conn.commit()
    except(Exception, psycopg2.DatabaseError) as error:
        print(error)
    finally:
        if conn is not None:
            conn.close()


    

#===================#
#     Peticiones    #
#===================#


#
#   ->  POSTS
#

    #   Registro de usuario ----------------- FUNCIONAL
@app.post("/signin") 
async def registerUser(userCredentials: UserCredentials):
    print("Se ha recibido la solicitud de registro del usuario " + userCredentials.username)
    
    # Verificar si el usuario ya existe
    existing_user = check_user_exists(userCredentials.username)
    
    if existing_user:
        raise HTTPException(status_code=400, detail="El usuario ya existe. Elija otro nombre de usuario.")
    
    # Insertar el nuevo usuario en la base de datos
    insert_user(userCredentials.username, userCredentials.password, userCredentials.token)
    
    return {"mensaje": "Usuario registrado exitosamente"}

def check_user_exists(username):
    sentenciaSQL = """SELECT * FROM "USER" WHERE "NAMEUSER" = %s"""
    conn = None
    
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL, (username,))
        existing_user = cur.fetchone()

        return existing_user is not None
    
    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al verificar la existencia del usuario")
    
    finally:
        if conn is not None:
            conn.close()

def insert_user(username, password, token):
    sentenciaSQL = """INSERT INTO "USER" ("NAMEUSER", "PASSWORDUSER", "TOKENUSER", "TIMESTAMPUSER") VALUES (%s, %s, %s, %s)"""
    conn = None
    
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        #Hora española en el momento del registro
        timestamp = datetime.now()
        timestamp_sp = timestamp + timedelta(hours = 1)

        cur.execute(sentenciaSQL, (username, password, token, timestamp_sp))
        conn.commit()

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al registrar el usuario")

    finally:
        if conn is not None:
            conn.close()



        #post de registro de intento de inicio de sesión
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        
        
        #post de incidentes ---------------- FUNCIONAL
@app.post("/incidents")
async def create_incident(incident: Incident):
    print("Se ha recibido un nuevo incidente")

    # Insertar el incidente en la base de datos
    sentenciaSQL = """INSERT INTO "INCIDENTS" ("TIMESTAMPINCIDENTS", "INCIDENTPIC", "SEVERITY") VALUES (%s, %s, %s)"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        #Hora española en el momento del incidente
        #Está en formato AA/MM/DD HH:MM:SS cambiar a --> DD/MM/AA
        timestamp = datetime.now()
        timestamp_sp = timestamp + timedelta(hours = 1)

        cur.execute(sentenciaSQL, (timestamp_sp, incident.incidentpic, incident.severity))
        conn.commit()

        return {"mensaje": "Incidente registrado exitosamente"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al registrar el incidente")

    finally:
        if conn is not None:
            conn.close()



    #   check usuario-contraseña ------------     +- FUNCIONAL
    

    #   En la consulta hay que introducir el token o dará error

@app.post("/login")
async def checkPassword(userCredentials : UserCredentials):
    print("Se ha recibido el intento de inicio de sesión por parte del usuario "+userCredentials.username)
    sentenciaSQL="""SELECT * FROM "USER" WHERE "NAMEUSER" = %s AND "PASSWORDUSER" = %s AND "TOKENUSER" = %s"""
    conn = None
    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL,(userCredentials.username,userCredentials.password, userCredentials.token))
        user= cur.fetchone()
        cur.close()

        if user:
            return {"mensaje":"Credenciales válidas"}
        else:
            return HTTPException(status_code=401, detail="Credenciales inválidas")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)
    finally:
        if conn is not None:
            conn.close()


    #   publicación horarios vigilancia

    #   En postman sólo se completa la consulta cuando meto todos los datos del usuario al mismo tiempo
    #   que los horarios de inicio y fin.

    #   Funciona algo del tipo:
    #   CONSULTA POSTMAN:
    #   {
    #   "userCredentials": {
    #       "username": "9960cb0c-8600-477c-a7c3-12f494968cf0", <-- uuid que se genera, no se conoce de ante mano
    #       "password": "alcoba",
    #       "token" : "askbd9v349f"
    #   },
    #   "schedule": {
    #       "scheduleStart": "2023-11-30",
    #       "scheduleEnd": "2023-12-2"
    #   }
    #   }

    #   Tampoco comprueba que el usuario esté registrado, simplemente crea el horario con los datos correctos
    
@app.post("/{user}/schedule")
async def create_schedule(userCredentials : UserCredentials, schedule : Schedule):
    print("Se ha recibido una publicación de horario por parte de: "+ userCredentials.username)
    sentenciaSQL="""INSERT INTO "WATCHSCHEDULE" ("SCHEDULESTART", "SCHEDULEEND", "SCHEDULEUSER") VALUES (%s, %s, %s)"""
    conn = None
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        #En username pide el uuid para funcionar, habría que añadir y generar en la estructura del user
        cur.execute(sentenciaSQL, (schedule.scheduleStart, schedule.scheduleEnd, userCredentials.username))
        conn.commit() 

        cur.close()

        return {"mensaje": "Horario subido correctamente"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=400, detail="Error al subir el horario")

    finally:
        if conn is not None:
            conn.close()



#
#   ->  GETS
#

    #   Consulta del horario del usuario
@app.get("/{user}/timetables")
async def check_schedule():
    print("Se han solicitado los horarios del usuario")
    sentenciaSQL="""SELECT * FROM "WATCHSCHEDULE" WHERE "SCHEDULEUSER" = %s"""
    conn = None
    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL,(user))
        user= cur.fetchone()
        cur.close()

        if user:
            return {"mensaje":"Credenciales válidas"}
        else:
            return HTTPException(status_code=401, detail="Credenciales inválidas")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)
    finally:
        if conn is not None:
            conn.close()


    #TO-DO
    
    #   get de incidentes //////////////////////////////////////////// NO ENTIENDO BIEN QUÉ HAY QUE HACER AQUÍ
@app.get("/incidents")
async def check_incidents():
    print("Se han solicitado los incidentes del sistema correspondientes al usuario")
    sentenciaSQL="""SELECT * FROM "INCIDENTS" """ #Se ven los incidentes del usuario seleccionado
    conn = None
    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL)
        user= cur.fetchone()
        cur.close()

        if user:
            return {"mensaje":"Credenciales válidas"}
        else:
            return HTTPException(status_code=401, detail="Credenciales inválidas")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)
    finally:
        if conn is not None:
            conn.close()



#========================#
#     Actualizaciones    #
#========================#

#
#   ->  PUTS
#

    #   Edición de los horarios de vigilancia
@app.put("/{user}/schedule")
async def edit_schedule(schedule : Schedule):
    print("Se ha recibido una edición de horario por parte de: "+ user)
    sentenciaSQL="""UPDATE "WATCHSCHEDULE" WHERE "SCHEDULEUSER" = %s
                        SET "SCHEDULESTART" = %s,
                        SET "SCHEDULEEND" = %s"""
    conn = None
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL, (schedule.scheduleStart, schedule.scheduleEnd, user))
        conn.commit() 

        cur.close()

        return {"mensaje": "Horario actualizado correctamente"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=400, detail="Error al editar el horario")

    finally:
        if conn is not None:
            conn.close()




#======================#
#     Eliminaciones    #
#======================#

#
#   ->  DELETES
#

    #   Eliminar un horario ¿del usuario que ha realizado la acción?
@app.delete("/{user}/schedule")
async def delete_schedule(schedule : int):
    print("Se ha recibido una eliminación de horario por parte de: "+ user)
    sentenciaSQL="""DELETE FROM "WATCHSCHEDULE" WHERE "IDSCHEDULE"= %s"""
    conn = None
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL, (user))
        conn.commit() 

        cur.close()

        return {"mensaje": "Horario eliminado correctamente"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=400, detail="Error al eliminar el horario")

    finally:
        if conn is not None:
            conn.close()



if __name__ == '__main__':
    import uvicorn
    print('\n\n\n\n')
    connect()
    uvicorn.run(app, host="0.0.0.0", port=8000)
    
