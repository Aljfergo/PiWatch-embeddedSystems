<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(loginAttempt, index) in loginAttempts" :key="index" cols="12" md="4">
          <v-card class="card-style">
            <v-card-title class="card-title-style">Intento de inicio de sesión {{ index+1 }}</v-card-title>
            <v-card-text class="card-text-style">Contraseña intentada: {{ loginAttempt.password }}</v-card-text>
            <v-card-text class="card-text-style">Hora del intento: {{ loginAttempt.timestamp }}</v-card-text>
            <v-card-text class="card-text-style">IP: {{ loginAttempt.ip }}</v-card-text>
          </v-card>
        </v-row>
      </v-row>
    </v-container>
  </v-sheet>
</template>

<script>
export default {
  props: {
    loginAttempts: {
      type: Array,
      required: true,
    },
  },
};
</script>

<style scoped>
.card-container-sheet {
  max-height: 450px;
  overflow-y: auto;
  overflow-x: hidden;
  display: flex;
  flex-direction: column;
  background-color:#c7e5d5;
}

.card-style {
  border: 1px solid #000000;
  background-color: #f3fff1;
  border-radius: 8px;
  margin: 16px;
  padding: 16px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  width:1000px
}

.card-title-style {
  font-size: 18px;
  font-weight: bold;
  margin-bottom: 8px;
}

.card-text-style {
  font-size: 14px;
  color: #555;
}
</style>
