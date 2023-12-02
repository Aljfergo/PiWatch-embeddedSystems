<template>
    <v-container>
      <v-row>
        <!-- Botón para crear nuevo horario -->
        <v-col>
          <v-btn @click="mostrarDialogo">Crear Nuevo Horario</v-btn>
        </v-col>
      </v-row>
  
      <!-- Diálogo para seleccionar fecha y hora -->
      <v-dialog v-model="dialogoVisible" max-width="80%" class="d-flex" style="display: flex; flex-direction: row;">
        <v-card>
          <v-card-title>Seleccionar Rango de Fechas y Horas</v-card-title>
  
          <v-card-text>
            <!-- Sección para la fecha y hora inicial -->
            <v-row>
            <v-col>
              <v-row>
                <v-date-picker v-model="fechaInicial" :min="fechaMinima" title="Inicio de vigilancia"></v-date-picker>
              </v-row>

              <v-row style="width: 80%">
                <v-select v-model="horaInicial" :items="horasValidas" :disabled="!fechaInicial" label="Hora"></v-select>
                <div>:</div>
              
                <v-select
                  v-model="minutosIniciales"
                  :items="minutosValidos"
                  label="Minutos"
                  :disabled="!fechaInicial"
                ></v-select>
              </v-row>
            </v-col>
  
   
            <v-divider vertical="true"></v-divider>
  
   
            <v-col>
              <v-row>
                <v-date-picker v-model="fechaFinal" :min="fechaInicial" title="Final de vigilancia" :disabled="!fechaInicial"></v-date-picker>
              </v-row>

              <v-row style="width:80%;">
                <v-select v-model="horaFinal" :items="horasValidas" label="Hora" :disabled="!fechaFinal"></v-select>
                <div>:</div>
              
              
                <v-select
                  v-model="minutosFinales"
                  :items="minutosValidos"
                  label="Minutos"
                  :disabled="!fechaFinal"
                ></v-select>
              </v-row>
            </v-col>
            </v-row>
          </v-card-text>
  
          <v-card-actions>
            <v-btn @click="guardarHorario">Guardar</v-btn>
            <v-btn @click="cerrarDialogo">Cancelar</v-btn>
          </v-card-actions>
        </v-card>
      </v-dialog>
    </v-container>
  </template>
  
  <script>
  export default {
    data() {
      return {
        dialogoVisible: false,
        fechaInicial: null,
        horaInicial: null,
        minutosIniciales: null,
        fechaFinal: null,
        horaFinal: null,
        minutosFinales: null,
        horasValidas: Array.from({ length: 24 }, (_, i) => String(i).padStart(2, '0')),
        minutosValidos: Array.from({ length: 60 }, (_, i) => String(i).padStart(2, '0'))
      };
    },
    computed: {
      fechaMinima() {
        const fechaActual = new Date();
        return fechaActual.toISOString().split('T')[0];
      },
    },
    methods: {
      mostrarDialogo() {
        this.dialogoVisible = true;
      },
      cerrarDialogo() {
        this.dialogoVisible = false;
        this.fechaInicial = null;
        this.horaInicial = null;
        this.minutosIniciales = null;
        this.fechaFinal = null;
        this.horaFinal = null;
        this.minutosFinales = null;
      },
      guardarHorario() {
        
        console.log("Fecha y hora inicial:", this.fechaInicial, this.horaInicial, this.minutosIniciales);
        console.log("Fecha y hora final:", this.fechaFinal, this.horaFinal, this.minutosFinales);
        this.cerrarDialogo();
      },
    },
  };
  </script>
  