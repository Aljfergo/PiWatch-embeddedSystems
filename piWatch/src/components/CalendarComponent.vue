<template>
    <div class="fade-background">
      <div class="fixed-button">
        <router-link :to="`/home/${user}`">
          <v-btn>Atrás</v-btn>
        </router-link>
      </div>
      <v-row style="align-self: center; text-align: center">
        <v-card style="margin-bottom:-20px; margin-left: 260px; margin-top: 30px; padding: 40px; width:1200px; background-color:#4b1139; color: whitesmoke;">
          <h1>
            <v-icon
              icon="mdi-clock"

              color="white"
              />
            
            Horarios 

            <v-icon
              icon="mdi-clock"

              color="white"
              />
          </h1>
        </v-card>
      </v-row>
      <v-row>
        <v-container style="background-color:#4f13365e;  border-radius: 10px; width: 1500px; padding: 3%; margin-top: 2%;">  
          <schedule-container :schedules="schedules" />
          <v-btn style="width: 1100px; font-size: xx-large;  margin-top:20px; border-radius: 8px; height: 150px; background-color: rgba(97, 37, 78, 0.562); color: whitesmoke;" @click="mostrarDialogo">+</v-btn>
        </v-container>
        
          
        
      </v-row>
      

      <v-dialog v-model="dialogoVisible" max-width="80%" class="d-flex" style="display: flex; flex-direction: row;">
        <v-card>
          <v-card-title>Seleccionar Rango de Fechas y Horas</v-card-title>
          <v-card-text>
            <v-row>
            <v-col>
              <v-row>
                <v-date-picker v-model="fechaInicial" @change="reformatearFechaInicial" :min="fechaMinima" title="Inicio de vigilancia"></v-date-picker>
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
  
   
            <v-divider vertical=true></v-divider>
  
   
            <v-col>
              <v-row>
                <v-date-picker v-model="fechaFinal" :min="fechaInicial" @change="reformatearFechaFinal" title="Final de vigilancia" :disabled="!fechaInicial"></v-date-picker>
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
            <v-btn @click="guardarHorario(user)">Guardar</v-btn>
            <v-btn @click="cerrarDialogo">Cancelar</v-btn>
          </v-card-actions>
        </v-card>
      </v-dialog>
    </div>
  </template>

  <style scoped>
  .fade-background{
    background: linear-gradient(70deg, #f3f4ca, rgb(57, 2, 48));
  }
  .fixed-button {
  position: fixed;
  bottom: 20px; 
  right: 20px; 
  z-index: 1000; 
}


</style>
  
  <script>
import ScheduleContainer from './scheduleContainer.vue';
import axios from 'axios';
import { format } from 'date-fns';
  export default {
    data() {
        return {
            schedules: [],
            dialogoVisible: false,
            fechaInicial: null,
            horaInicial: null,
            minutosIniciales: null,
            fechaFinal: null,
            horaFinal: null,
            minutosFinales: null,
            horasValidas: Array.from({ length: 24 }, (_, i) => String(i).padStart(2, '0')),
            minutosValidos: Array.from({ length: 60 }, (_, i) => String(i).padStart(2, '0')),
            user: this.$route.params.user,
            fechaFinalFormateada: '',
            fechaInicialFormateada: '',
        };
    },
    mounted() {
      const user = this.$route.params.user;
      this.fetchCards(user);
      
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
        guardarHorario(user) {

            if (!this.fechaInicial || !this.horaInicial || !this.minutosIniciales || !this.fechaFinal || !this.horaFinal || !this.minutosFinales) {
                console.error('Por favor, complete todas las selecciones antes de guardar.');
                return;
            }
            this.reformatearFechaFinal();
            this.reformatearFechaInicial();
            const scheduleStart = `${this.fechaInicialFormateada} ${this.horaInicial}:${this.minutosIniciales}`;
            const scheduleEnd = `${this.fechaFinalFormateada} ${this.horaFinal}:${this.minutosFinales}`;

            axios.post(`http://localhost:8000/${user}/schedule`, {
                scheduleStart,
                scheduleEnd,
            })
                .then(response => {
                console.log('Horario guardado exitosamente:', response.data);
                this.cerrarDialogo();
                location.reload();
            })
                .catch(error => {
                console.error('Error al guardar el horario:', error);
            });
        },
        async fetchCards(user) {
            try {
                const response = await axios.get(`http://localhost:8000/${user}/schedule`);
                this.schedules = response.data;
                console.log(this.schedules)
            }
            catch (error) {
                console.error('Error al obtener los horarios', error);
            }
        },
        formatDate (date) {
          if (!date) return null
          const [year, month, day] = date.split('-')
          return `${day}-${month}-${year}`
        },

        reformatearFechaInicial() {
          console.log("llega");
          if (this.fechaInicial) {
            this.fechaInicialFormateada = format(this.fechaInicial, 'yyyy-MM-dd');
          }
        },

        reformatearFechaFinal() {
          console.log("llega");
          if (this.fechaFinal) {
            this.fechaFinalFormateada = format(this.fechaFinal, 'yyyy-MM-dd');
          }
        }
    },
    components: { ScheduleContainer }
};
  </script>
  