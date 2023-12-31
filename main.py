from fastapi import FastAPI, HTTPException, Request
import psycopg2
from config import config
from pydantic import BaseModel
from colorama import Style, Fore, Back, init
from fastapi.middleware.cors import CORSMiddleware
from datetime import datetime, timedelta
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
import socket

app=FastAPI()



#   Configuración de las políticas de CORS (todo permitido)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_methods=["*"],  
    allow_headers=["*"], 
    allow_credentials=True, 
    expose_headers=["*"],  
)



#   Base models para los bodies
class UserCredentialsSignin(BaseModel):
    username: str
    password: str
    token: str

class UserCredentialsLogin(BaseModel):
    username: str
    password: str

class Schedule(BaseModel):
    scheduleStart: str
    scheduleEnd: str

class Incident(BaseModel):
    incidentPic: str
    severity: int



#   Conexión inicial
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

        #with open("bdd/piwatch_structure.sql", "r") as script_file:
        #    cur.execute(script_file.read())
        #    print("Tablas creadas con éxito")
        
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
async def registerUser(userCredentialsSignin: UserCredentialsSignin):
    print("Se ha recibido la solicitud de registro del usuario " + userCredentialsSignin.username)
    
    # Verificar si el usuario ya existe
    existing_user = check_user_exists(userCredentialsSignin.username)
    
    if existing_user:
        raise HTTPException(status_code=400, detail="El usuario ya existe. Elija otro nombre de usuario.")
    
    # Insertar el nuevo usuario en la base de datos
    insert_user(userCredentialsSignin.username, userCredentialsSignin.password, userCredentialsSignin.token)
    
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



#   check usuario-contraseña ------------ FUNCIONAL
@app.post("/login")
async def checkPassword(userCredentialsLogin: UserCredentialsLogin):
    print("Se ha recibido el intento de inicio de sesión por parte del usuario " + userCredentialsLogin.username)

    sentenciaSQL="""SELECT * FROM "USER" WHERE "NAMEUSER" = %s AND "PASSWORDUSER" = %s"""
    conn = None

    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL, (userCredentialsLogin.username, userCredentialsLogin.password))
        user= cur.fetchall()
        conn.commit()

        if user:
            return {"mensaje": "Credenciales válidas"}
        else:
            return HTTPException(status=401, detail="Credenciales inválidas")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)

    finally:
        if conn is not None:
            conn.close()



#   Middleware para obtener la dirección IP del cliente que intenta registrarse
@app.middleware("http")
async def add_client_ip(request: Request, call_next):
    client_host = request.client.host

    # Intenta obtener la dirección IP real del cliente desde la cabecera X-Forwarded-For
    forwarded_for = request.headers.get("X-Forwarded-For")
    if forwarded_for:
        client_ip = forwarded_for.split(",")[0].strip()
    else:
        client_ip = client_host

    request.state.client_ip = client_ip
    
    response = await call_next(request)
    
    return response

#   Registro de intento de inicio de sesión ------------ FUNCIONAL
#   *************** Mirar si se puede obtener la IP real, no la pública
@app.post("/loginAttempts")
async def regist_login(userCredentialsLogin: UserCredentialsLogin, request: Request):
    # Obtenemos la dirección IP del cliente desde el middleware
    client_ip = request.state.client_ip
    
    print("El usuario " + userCredentialsLogin.username + " ha intentado iniciar sesión desde la IP " + client_ip)

    sentenciaSQL = """INSERT INTO "LOGINATTEMPT" ("NAMELOGIN", "PASSWORDLOGIN", "TIMESTAMPLOGIN", "IP") VALUES (%s, %s, %s, %s)"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        # Hora española en el momento del intento de inicio de sesión
        timestamp = datetime.now()
        timestamp_sp = timestamp + timedelta(hours=1)

        cur.execute(sentenciaSQL, (userCredentialsLogin.username, userCredentialsLogin.password, timestamp_sp, client_ip))
        conn.commit()

        return {"mensaje": "Intento de inicio de sesión registrado"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al registrar el inicio de sesión")

    finally:
        if conn is not None:
            conn.close()

        
        
#   post de incidentes ---------------- FUNCIONAL
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

        cur.execute(sentenciaSQL, (timestamp_sp, incident.incidentPic, incident.severity))
        conn.commit()

        return {"mensaje": "Incidente registrado exitosamente"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al registrar el incidente")

    finally:
        if conn is not None:
            conn.close()


    
#   publicación horarios vigilancia --------------- FUNCIONAL
@app.post("/{user}/schedule")
async def create_schedule(user: str, schedule: Schedule):
    print("Se ha recibido una publicación de horario por parte de: " + user)

    sentenciaSQL="""INSERT INTO "WATCHSCHEDULE" ("SCHEDULESTART", "SCHEDULEEND", "SCHEDULEUSER") VALUES (%s, %s, %s)"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        #En username pide el uuid para funcionar, habría que añadir y generar en la estructura del user
        cur.execute(sentenciaSQL, (schedule.scheduleStart, schedule.scheduleEnd, user))
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
@app.get("/{user}/schedule")
async def check_schedule(user: str):
    print("Se han solicitado los horarios del usuario " + user)
    
    sentenciaSQL="""SELECT * FROM "WATCHSCHEDULE" WHERE "SCHEDULEUSER" = %s ORDER BY "SCHEDULESTART" ASC"""
    conn = None

    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL,(user,))
        sched= cur.fetchall()
        cur.close()

        if sched:
            return [{"id": sch[0], "scheduleStart": sch[1], "scheduleEnd": sch[2], "active": sch[4]} for sch in sched]

        else:
            return HTTPException(status_code=401, detail="El usuario solicitado no existe")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al recuperar los horarios")

    finally:
        if conn is not None:
            conn.close()


    
    #   get de incidentes --------   FUNCIONAL
@app.get("/incidents")
async def check_incidents():
    print("Se han solicitado los incidentes registrados por el sistema")

    sentenciaSQL="""SELECT * FROM "INCIDENTS" """ #Se ven los incidentes del usuario seleccionado
    conn = None

    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL)
        incidents = cur.fetchall()  # Obtener todos los resultados
        cur.close()

        if incidents:
            # Devolver los incidentes en el formato deseado, por ejemplo, como una lista de diccionarios
            return [{"id": incident[0], "timestamp": incident[1], "imagepic": incident[2], "severity": incident[3]} for incident in incidents]
        else:
            return {"mensaje": "No hay incidentes"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error interno del servidor")

    finally:
        if conn is not None:
            conn.close()



    # GET -- Devuelve el token del user cuyo horario está activo

@app.get("/activeScheduleUserToken")
async def check_token():
    sentenciaSQL_watchschedule = """SELECT "SCHEDULEUSER" FROM "WATCHSCHEDULE" WHERE "ACTIVE" = TRUE LIMIT 1"""
    sentenciaSQL_user = """SELECT "TOKENUSER" FROM "USER" WHERE "NAMEUSER" = %s"""

    conn = None

    try:
        
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        
        cur.execute(sentenciaSQL_watchschedule)
        watchschedule_user = cur.fetchone()

        if not watchschedule_user:
            raise HTTPException(status_code=404, detail="No hay usuarios activos en WATCHSCHEDULE")

        
        username = watchschedule_user[0]
        cur.execute(sentenciaSQL_user, (username,))
        token = cur.fetchone()

        cur.close()

        if token:
            return token[0]
        else:
            raise HTTPException(status_code=404, detail=f"No se encontró el usuario {username} en la tabla USER")

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error interno del servidor")

    finally:
        if conn is not None:
            conn.close()



#	Devolver el horario activo en el instante de tiempo
@app.get("/activeSchedule")
async def check_activeSchedule():
    print("Se ha solicitado el horario activo")
    
    sentenciaSQL="""SELECT * FROM "WATCHSCHEDULE" WHERE "ACTIVE" = TRUE"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL)
        sched = cur.fetchone()
        cur.close()

        if sched:
            return {"scheduleStart": sched[1], "scheduleEnd": sched[2]}

        else:
            return HTTPException(status_code=401, detail="No hay ningún horario activo")
        
    except (Exception, psycopg2.DatabaseError)  as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error en la base de datos al recuperar el horario")

    finally:
        if conn is not None:
            conn.close()



#   GET loginAttempts de un usuario
@app.get("/{user}/loginAttempts")
async def check_login(user: str):
    sentenciaSQL = """SELECT * FROM "LOGINATTEMPT" WHERE "NAMELOGIN"= %s"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL, (user, ))
        attempts = cur.fetchall()
        conn.commit()

        if attempts:
            return [{"id": attempt[0], "password": attempt[2], "timestamp": attempt[3], "ip": attempt[4]} for attempt in attempts]
    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=500, detail="Error al recuperar los intentos de inicio de sesión del usuario")

    finally:
        if conn is not None:
            conn.close()



#========================#
#     Actualizaciones    #
#========================#

#
#   ->  PUTS
#

#   Edición de los horarios de vigilancia ---------- FUNCIONAL
# *********************** OPCIONAL. Función coincide con el id del horario no con el usuario para mayot eficacia
@app.put("/{user}/schedule")
async def edit_schedule(user: str, schedule: Schedule):
    print("Se ha recibido una edición de horario por parte de: " + user)

    sentenciaSQL="""UPDATE "WATCHSCHEDULE"
                        SET "SCHEDULESTART" = %s, "SCHEDULEEND" = %s
                        WHERE "SCHEDULEUSER" = %s"""
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



#   Edita el estado del horario -- ACTIVO / INACTIVO
@app.put("/schedule/{schedule_id}")
async def edit_schedule(schedule_id: str):
    print("Se ha recibido una petición de cambio de estado para el horario: " + schedule_id)

    sentenciaSQLActiva = """UPDATE "WATCHSCHEDULE"
                        SET "ACTIVE" = TRUE
                        WHERE "IDSCHEDULE" = %s""" #setea activo el horario del id

    sentenciaSQLDesactiva = """UPDATE "WATCHSCHEDULE"
                            SET "ACTIVE" = FALSE
                            WHERE "ACTIVE" = TRUE"""

    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQLDesactiva)
        cur.execute(sentenciaSQLActiva, (schedule_id,))
        conn.commit() 

        cur.close()

        return {"mensaje": "Horario activo"}

    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
        raise HTTPException(status_code=400, detail="Error al cambiar el estado de el/los horario/s")

    finally:
        if conn is not None:
            conn.close()


#======================#
#     Eliminaciones    #
#======================#

#
#   ->  DELETES
#

#   Eliminar un horario 
@app.delete("/schedule/{schedule_id}")
async def delete_schedule(schedule_id: str):
    print("Se ha recibido una petición de eliminación del horario " + schedule_id)

    sentenciaSQL="""DELETE FROM "WATCHSCHEDULE" WHERE "IDSCHEDULE"= %s"""
    conn = None

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

        cur.execute(sentenciaSQL, (schedule_id,))
        sched : cur.fetchone()
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
