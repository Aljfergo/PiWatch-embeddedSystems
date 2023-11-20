from fastapi import FastAPI, HTTPException
import psycopg2
from config import config
from pydantic import BaseModel
from colorama import Style,Fore, Back, init
from fastapi.middleware.cors import CORSMiddleware

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


class Schedule(BaseModel):
    scheduleStart: str
    scheduleEnd: str




#
#   Conexión inicial
#

def connect():
    conn=None
    
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
        cur.close()
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

    #   check usuario-contraseña
@app.post("/login")
async def checkPassword(userCredentials : UserCredentials):
    print("Se ha recibido el intento de inicio de sesión por parte del usuario "+userCredentials.username)
    sentenciaSQL="""SELECT * FROM users WHERE username = %s AND password = %s"""
    conn = None
    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL,(userCredentials.username,userCredentials.password))
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
@app.post("/{user}/schedule")
async def checkPassword(schedule : Schedule):
    print("Se ha recibido una publicación de horario por parte de: "+ user)
    sentenciaSQL="""INSERT INTO "WATCHSCHEDULE" ("SCHEDULESTART", "SCHEDULEEND", "SCHEDULEUSER") VALUES (%s, %s, %s)"""
    conn = None
    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()

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
@app.get("/{user}/timetables")
async def checkData():
    print("Se han solicitado los horarios del usuario")
    sentenciaSQL="""SELECT * FROM WATCHSCHEDULE WHERE SCHEDULEUSER = %s"""
    conn = None
    try:
        params=config()
        conn =psycopg2.connect(**params)
        cur=conn.cursor()

        cur.execute(sentenciaSQL,(user)))
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



if __name__ == '__main__':
    import uvicorn
    print('\n\n\n\n')
    connect()
    uvicorn.run(app, host="0.0.0.0", port=8000)
    
