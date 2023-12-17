<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(schedule, index) in schedules" :key="index" cols="12" md="4">
          <v-card class="card-style">
            <v-col>
            <v-card-title>Inicio de vigilancia: {{ schedule.scheduleEnd }}</v-card-title>
            <v-card-title>Fin de vigilancia: {{ schedule.scheduleStart }}</v-card-title>
           </v-col>
           <v-col>
              <v-btn @click="enableSchedule(schedule.id)" :disabled="schedule.active" style="height: 100px; margin-left: 180px; width: 300px;">
              {{ schedule.active ? 'Activado' : 'Activar horario' }}
             
             </v-btn>
            </v-col>
             <v-col>
            
              <v-btn @click="deleteSchedule(schedule.id)" style="height: 100px; width: 100px; margin-right: -100px;">
              <v-icon
              icon="mdi-trash-can"
              size="large"
              color="grey"
              />
            </v-btn>
          </v-col>
          </v-card>
        </v-row>
      </v-row>
    </v-container>
  </v-sheet>
</template>

<script>
import axios from 'axios';
export default {
  props: {
    schedules: {
      type: Array,
      required: true,
    },
  },
  methods: {
    enableSchedule(scheduleId) {
      axios.put(`http://localhost:8000/schedule/${scheduleId}`, {}, {
        headers: {
          'Content-Type': 'application/json',
        },
      })
      .then(response => {
        console.log('Schedule activado correctamente');
          // Si quieres recargar la página solo después de que la petición sea exitosa, mueve location.reload() aquí
        location.reload();
      })
      .catch(error => {
        console.error('Error al activar el horario:', error);
      });

        
    },

    deleteSchedule(scheduleId) {
      axios.delete(`http://localhost:8000/schedule/${scheduleId}`, {}, {
        headers: {
          'Content-Type': 'application/json',
        },
      })
      .then(response => {
        console.log('Schedule borrado correctamente');
      
        location.reload();
      })
      .catch(error => {
        console.error('Error al borrar el horario:', error);
      });

        
    },
  },
};
</script>

<style scoped>
.card-container-sheet {
  max-height: 350px;
  overflow-y: auto;
  overflow-x: hidden;
  background-color: #fff6df00;
  margin-right: -30px;
}

.card-style { 
  border: 1px solid #000000;
  background-color: #4b1139;
  color: whitesmoke;
  border-radius: 8px;
  margin: 10px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  width:1100px;
  display:flex;
}
</style>