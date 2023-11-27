CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE "USER"
(
    "IDUSER"                   uuid DEFAULT uuid_generate_v4(),
    "NAMEUSER"                 VARCHAR(100) NOT NULL,
    "PASSWORDUSER"             VARCHAR(100) NOT NULL,
    "TIMESTAMPUSER"            TIMESTAMP    NOT NULL,
    PRIMARY KEY ("IDUSER")
);

CREATE TABLE "LOGINATTEMPT"
(
    "IDLOGIN"                   uuid DEFAULT uuid_generate_v4(),
    "NAMELOGIN"                 VARCHAR(100) NOT NULL,
    "PASSWORDLOGIN"             VARCHAR(100) NOT NULL,
    "TIMESTAMPLOGIN"            TIMESTAMP    NOT NULL,
    "IP"                        VARCHAR(20)  NOT NULL,
    PRIMARY KEY ("IDLOGIN")
);

CREATE TABLE "INCIDENTS"
(
    "IDINCIDENT"               uuid DEFAULT uuid_generate_v4(),
    "TIMESTAMPINCIDENTS"       INT NOT NULL,
    "INCIDENTPIC"              VARCHAR(50) DEFAULT NULL,
    "SEVERITY"                 INT DEFAULT NULL,   
    PRIMARY KEY ("IDINCIDENT")
);

CREATE TABLE "WATCHSCHEDULE" (
  "IDSCHEDULE"                uuid DEFAULT uuid_generate_v4(),
  "SCHEDULESTART"             TIMESTAMP NOT NULL,
  "SCHEDULEEND"               TIMESTAMP NOT NULL,
  "SCHEDULEUSER"              uuid DEFAULT NULL,
  PRIMARY KEY ("IDSCHEDULE")
);
