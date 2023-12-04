CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE IF NOT EXISTS "USER"
(
    "IDUSER"                   uuid DEFAULT uuid_generate_v4(),
    "NAMEUSER"                 VARCHAR(100) NOT NULL,
    "PASSWORDUSER"             VARCHAR(100) NOT NULL,
    "TOKENUSER"                VARCHAR(100) NOT NULL,
    "TIMESTAMPUSER"            TIMESTAMP    NOT NULL,
    PRIMARY KEY ("IDUSER")
);

CREATE TABLE IF NOT EXISTS "LOGINATTEMPT"
(
    "IDLOGIN"                   uuid DEFAULT uuid_generate_v4(),
    "NAMELOGIN"                 VARCHAR(100) NOT NULL,
    "PASSWORDLOGIN"             VARCHAR(100) NOT NULL,
    "TIMESTAMPLOGIN"            TIMESTAMP    NOT NULL,
    --"ESTADO"                    VARCHAR(100) NOT NULL,   
    "IP"                        VARCHAR(20)  NOT NULL,
    PRIMARY KEY ("IDLOGIN")
);

CREATE TABLE IF NOT EXISTS "INCIDENTS"
(
    "IDINCIDENT"               uuid DEFAULT uuid_generate_v4(),
    "TIMESTAMPINCIDENTS"       TIMESTAMP NOT NULL,
    "INCIDENTPIC"              VARCHAR(50) DEFAULT NULL,
    "SEVERITY"                 INT DEFAULT NULL,   
    PRIMARY KEY ("IDINCIDENT")
);

CREATE TABLE IF NOT EXISTS "WATCHSCHEDULE" (
  "IDSCHEDULE"                uuid DEFAULT uuid_generate_v4(),
  "SCHEDULESTART"             TIMESTAMP NOT NULL,
  "SCHEDULEEND"               TIMESTAMP NOT NULL,
  "SCHEDULEUSER"              VARCHAR(100) NOT NULL,
  PRIMARY KEY ("IDSCHEDULE")
);
