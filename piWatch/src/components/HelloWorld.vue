<template>
  <div class="fade-background">
  <v-container style="background-color:rgba(255, 255, 240, 0.407); border-radius: 10px; width: 50%; padding: 3%; margin-top: 20px;">
    <v-responsive class="align-center text-center fill-height">
      
        <v-img height="250" src="@/assets/logo.png" style="margin-right: 10px;" />
        <div>
          <div class="text-body-2 font-weight-light mb-n1" style="margin-bottom: 5px;">Bienvenido a</div>
          <h1 class="text-h2 font-weight-bold" style="margin-bottom: 5px;">PiWatch 👁</h1>
          <div class="py-14"></div>
        </div>
      
      <v-row class="d-flex align-center justify-center" style="margin-top: -10%;">
        <v-col cols="auto">

          <router-link :to="`/schedules/${user}`">
            <v-btn
              color=#4b1139
              min-width="228"
              rel="noopener noreferrer"
              size="x-large"
              target="_blank"
              variant="flat"
            >
              <v-icon
                icon="mdi-clock-outline"
                size="large"
                start
              />

            Registre un horario
          </v-btn>
        </router-link>
        </v-col>

        <v-col cols="auto">
          <router-link :to="`/incidents/${user}`">
            <v-btn
                color=#2e0b51
                min-width="228"
                rel="noopener noreferrer"
                size="x-large"
                target="_blank"
                variant="flat"
              >
                <v-icon
                  icon="mdi-alert-outline"
                  size="large"
                  start
                />

              Consulte los incidentes
            </v-btn>
          </router-link>
        </v-col>

          <v-col cols="auto">
            <router-link :to="`/loginAttempts/${user}`">
              <v-btn
                color=#38503a
                min-width="228"
                rel="noopener noreferrer"
                size="x-large"
                target="_blank"
                variant="flat"
              >
                <v-icon
                  icon="mdi-magnify"
                  size="large"
                  start
                />

              Inicios de sesion
            </v-btn>
            </router-link>
          </v-col>
        </v-row>
      </v-responsive>
    </v-container>
  </div>
</template>
<style scoped>
  .fade-background{
    background: linear-gradient(70deg, #f3f4ca, rgb(57, 2, 48));
    height: 830px !important;
    margin-top: -50px;
    padding: 100px;
  }

</style>
<script>
  export default {
    data() {
        return {
            
            user: this.$route.params.user,
        };
    },
    mounted() {
      const user = this.$route.params.user;
      
      
    },
    methods: {
        
  
    },
};
  
</script>
