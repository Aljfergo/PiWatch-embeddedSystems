<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(incident, index) in incidents" :key="index" cols="12" md="4">
          <v-card>
            <v-card-title>{{ incident.severity }}</v-card-title>
            <v-card-title>{{ incident.timestamp }}</v-card-title>
            <v-card-title>{{ incident.incidentPic }}</v-card-title>
          </v-card>
        </v-row>
      </v-row>
    </v-container>
  </v-sheet>
</template>

<script>
export default {
  props: {
    incidents: {
      type: Array,
      required: true,
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