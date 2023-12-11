<template>
    <v-container>
      
      <v-row>
        <v-container style="background-color:#c58fb4; border-radius: 10px; width: 80%; padding: 3%; margin-top: 2%;">  
          <IncidentContainer :incidents="incidents" />
        </v-container>
      </v-row>
    </v-container>
  </template>
  
  <script>
import IncidentContainer from './IncidentContainer.vue';
import axios from 'axios';

  export default {
    data() {
        return {
            incidents: [],
        };
    },
    mounted() {
      
      this.fetchCards(user);
      
    },
    methods: {
        
        async fetchCards() {
            try {
                const response = await axios.get(`http://localhost:8000/incidents`);
                this.incidents = response.data;
                console.log(this.incidents)
            }
            catch (error) {
                console.error('Error al obtener los horarios', error);
            }
        },
    },
    components: { LoginAttemptContainer }
};
  </script>
  