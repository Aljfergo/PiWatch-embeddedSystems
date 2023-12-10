<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(schedule, index) in schedules" :key="index" cols="12" md="4">
          <v-card>
            <v-card-title>{{ schedule.scheduleEnd }}</v-card-title>
            <v-card-title>{{ schedule.scheduleStart }}</v-card-title>
            <v-btn @click="enableSchedule(schedule.id)" :disabled="schedule.active">
              {{ schedule.active ? 'Activado' : 'Activar horario' }}
            </v-btn>
          </v-card>
        </v-row>
      </v-row>
    </v-container>
  </v-sheet>
</template>

<script>
export default {
  props: {
    schedules: {
      type: Array,
      required: true,
    },
  },
  methods: {
    enableSchedule(scheduleId) {
      fetch(`/api/schedule/${scheduleId}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },

      })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Schedule activado correctamente');
        })
        .catch(error => {
          console.error('Error al activar el horario:', error);
        });

        location.reload();
    },
  },
};
</script>

<style scoped>
.card-container-sheet {
  max-height: 500px;
  overflow-y: auto;
}
</style>