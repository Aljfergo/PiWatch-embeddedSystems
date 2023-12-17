<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(incident, index) in incidents" :key="index" cols="12" md="4">
          <v-card :class="getCardColorClass(incident.severity)">
            <v-card-title>{{ incident.imagepic }}</v-card-title>
            <v-card-text>{{ getSeverityText(incident.severity) }}</v-card-text>
            <v-card-text>Hora de la intrusión: {{ incident.timestamp }}</v-card-text>
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

  methods: {
    getSeverityText(severity) {
      switch (severity) {
        case 0:
          return "Se ha detectado movimiento, pero nada raro fue detectado";
        case 1:
          return "Se ha detectado movimiento que proviene de un animal, ¿Se trata de tu mascota?";
        case 2:
          return "Se ha detectado movimiento que es posible que provenga de un ser humano, por favor revise la foto de la intrusión";
        case 3:
          return "SE HA DETECTADO UN INTRUSO EN LA ZONA VIGILADA";
        default:
          return "";
      }
    },
    getCardColorClass(severity) {
      return `color-${severity}`;
    },


  }
};
</script>

<style scoped>
  .card-container-sheet {
    max-height: 500px;
    overflow-y: auto;
    overflow-x: hidden;
    background-color: #bbc6ff;
  }

  .color-0 {
  border: 1px solid #000000;
  background-color: #f5f5f5; /* Blanco roto */
  color: #000; /* Texto negro o personalizado */
  width: 1500px;
  border-radius: 8px;
  margin: 16px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.color-1 {
  border: 1px solid #000000;
  background-color: #f9f3b8; /* Amarillo claro */
  color: #000; /* Texto negro o personalizado */
  width: 1500px;
  border-radius: 8px;
  margin: 16px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.color-2 {
  border: 1px solid #000000;
  background-color: #ffe0b2; /* Naranja claro */
  color: #000; /* Texto negro o personalizado */
  width: 1500px;
  border-radius: 8px;
  margin: 16px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.color-3 {
  border: 1px solid #000000;
  background-color: #ffcdd2; /* Rojo claro */
  color: #000; /* Texto negro o personalizado */
  width: 1500px;
  border-radius: 8px;
  margin: 16px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}
</style>