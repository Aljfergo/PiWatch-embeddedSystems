<template>
    <v-sheet class="card-container-sheet">
     <v-container fluid>
       <v-row>
         <v-col v-for="(schedule, index) in shares" :key="index" cols="12" md="4">
           <v-card>
             <v-card-title>{{ schedule.scheduleEnd }}</v-card-title>
             <v-card-title>{{ schedule.scheduleStart }}</v-card-title>
           </v-card>
         </v-col>
       </v-row>
     </v-container>
    </v-sheet>
   </template>
   
   <script>
   export default {
     props: {
       schedule: {
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