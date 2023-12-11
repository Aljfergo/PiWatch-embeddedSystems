<template>
  <v-sheet class="card-container-sheet">
    <v-container fluid>
      <v-row>
        <v-row v-for="(loginAttempt, index) in loginAttempts" :key="index" cols="12" md="4">
          <v-card>
            <v-card-title>{{ loginAttempt.password }}</v-card-title>
            <v-card-title>{{ loginAttempt.timestamp }}</v-card-title>
            <v-card-title>{{ loginAttempt.ip }}</v-card-title>
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
  max-height: 500px;
  overflow-y: auto;
}
</style>