<template>
   <div class="fade-background" >
      <div class="fixed-button">
        <router-link :to="`/home/${user}`">
          <v-btn>Atrás</v-btn>
        </router-link>
      </div>
      <v-row style="align-self: center; text-align: center">
        <v-card style="margin:20px; margin-left: 310px; margin-top: 60px; padding: 40px; width:1100px; background-color:#bbc6ff; color: black;">
          <h1>Incidentes ocurridos</h1>
        </v-card>
      </v-row>
    
      
      <v-row>
        <v-container style="background-color:#bbc6ff; border-radius: 10px; width: 1100px; padding: 3%; margin-top: 2%;">  
          <IncidentContainer :incidents="incidents" />
        </v-container>
      </v-row>



    </div>
  </template>
  <style scoped>
  .fade-background{
    background: linear-gradient(70deg, #b1aaff, #2e0b51);
    
  }
  .fixed-button {
  position: fixed;
  bottom: 20px; 
  right: 20px; 
  z-index: 1000; 
}

</style>
  <script>
import IncidentContainer from './IncidentContainer.vue';
import axios from 'axios';

  export default {
    data() {
        return {
            incidents: [],
            user: this.$route.params.user,
        };
    },
   
    mounted() {
      
      this.fetchCards();
      
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
    components: { IncidentContainer }
};
  </script>
  