<template>
  <v-container style="background-color:ivory; border-radius: 10px; width: 50%; padding: 3%; margin-top: 2%;">
    <v-responsive class="align-center text-center fill-height">
      
        <v-img height="250" src="@/assets/logo.png" style="margin-right: 10px;" />
        <div>
          <div class="text-body-2 font-weight-light mb-n1" style="margin-bottom: 5px;">Bienvenido a</div>
          <h1 class="text-h2 font-weight-bold" style="margin-bottom: 5px;">PiWatch 👁</h1>
          <div class="py-14"></div>
        </div>
      
      <v-row class="d-flex align-center justify-center" style="margin-top: -10%;">
        <v-col cols="auto">
          <v-btn
            color=#8b008b
            href="https://t.me/PiWatch_AlertBot"
            min-width="228"
            rel="noopener noreferrer"
            size="x-large"
            target="_blank"
            variant="flat"
          >
            <v-icon
              icon="mdi-speedometer"
              size="large"
              start
            />

            Empezar a usar
          </v-btn>
        </v-col>

        <v-col cols="auto">
          <v-btn
            href="https://api.telegram.org/bot6785587695:AAH0tjLxIIlGxuaUaJIXewNlkQO2HI9MeT4/getUpdates"
            min-width="164"
            rel="noopener noreferrer"
            target="_blank"
            variant="text"
          >
            <v-icon
              icon="mdi-account-group"
              size="large"
              start
            />

            Api
          </v-btn>
        </v-col>
      </v-row>
    </v-responsive>
  </v-container>
</template>

<script setup>
  //
</script>
