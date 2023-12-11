<template>
    <v-container>
      
      <v-row>
        <v-container style="background-color:#c58fb4; border-radius: 10px; width: 80%; padding: 3%; margin-top: 2%;">  
          <LoginAttemptContainer :loginAttempts="loginAttempts" />
        </v-container>
      </v-row>
    </v-container>
  </template>
  
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
  