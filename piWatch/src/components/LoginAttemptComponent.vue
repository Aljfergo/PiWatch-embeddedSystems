<template>
    <div class="fade-background" >
      <div class="fixed-button">
        <router-link :to="`/home/${user}`">
          <v-btn>Atrás</v-btn>
        </router-link>
      </div>
      <v-row style="align-self: center; text-align: center">
        <v-card style="margin:20px; margin-left: 310px; margin-top: 60px; padding: 40px; width:1100px; background-color:#c7e5d5;">
          <h1>Intentos de incio de sesión en tu cuenta</h1>
        </v-card>
      </v-row>
      <v-row>
        <v-container style="background-color:#c7e5d5; border-radius: 10px; width: 1100px; padding: 3%; margin-top: 2%;">  
          <LoginAttemptContainer :loginAttempts="loginAttempts" />
        </v-container>
      </v-row>
    </div>
  </template>
  <style scoped>
  .fade-background{
    background: linear-gradient(70deg, #38503a, rgb(166, 224, 152));
    
  }
  .fixed-button {
  position: fixed;
  bottom: 20px; 
  right: 20px; 
  z-index: 1000; 
}

</style>
  <script>
import LoginAttemptContainer from './LoginAttemptContainer.vue';
import axios from 'axios';

  export default {
    data() {
        return {
            loginAttempts: [],
            user: this.$route.params.user,
        };
    },
    mounted() {
      const user = this.$route.params.user;
      this.fetchCards(user);
      
    },
    methods: {
        
        async fetchCards(user) {
            try {
                const response = await axios.get(`http://localhost:8000/${user}/loginAttempts`);
                this.loginAttempts = response.data;
                console.log(this.loginAttempts)
            }
            catch (error) {
                console.error('Error al obtener los horarios', error);
            }
        },
    },
    components: { LoginAttemptContainer }
};
  </script>
  