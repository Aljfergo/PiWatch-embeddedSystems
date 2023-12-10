<template>
  <div class="centered-container fade-background">
    
    <transition name="fade">
        <v-card v-if="mostrarError" class="error">
          <h2>
            No se ha podido registrar al usuario
          </h2>
        </v-card>
    </transition>
    <v-row class="align-center justify-center" style="width:90%; height: 90%; display:flex;">
      
      <v-container class="component-card align-center">
        
          
            <h2 style="margin-bottom: 100px;margin-bottom: 20px; margin-top: -150px; text-transform: uppercase; font-size: 50px; font-family: Cambria, Cochin, Georgia, Times, 'Times New Roman', serif;">Bienvenido</h2>
              <v-text-field
               
                class="login-textfield"
                label="Nombre de usuario"
                rounded="xl"
                
                bg-color="#fff2e3"
                v-model="username"
                variant="outlined"
                
              >
              <template v-slot:label>
              
                <div style="font-size:20px; font-family:Verdana, Geneva, Tahoma, sans-serif; padding-left: 30px;">Nombre de usuario</div>
              
              </template>
              <template v-slot:style>
              
                { "font-size: 20px" }
            
              </template>
            </v-text-field>
              
            
              <v-text-field
                
                class="login-textfield"
                label="Contraseña"
                
                bg-color="#fff2e3"
                v-model="password"
                variant="outlined"
                rounded="xl"  
                :type="showPassword ? 'text' : 'password'"

                
              >  
                <template v-slot:append-inner>
                  <v-icon @click="toggleVisibility" class="icon-size">{{showPassword ? 'mdi-eye' : 'mdi-eye-off'}}</v-icon>
                </template>
                <template v-slot:label>
              
                  <div style="font-size:20px; font-family:Verdana, Geneva, Tahoma, sans-serif; padding-left: 30px;">Contraseña</div>
            
            </template>
                
              </v-text-field>

              <v-text-field
                
                class="login-textfield"
                label="Repite la contraseña"
                
                bg-color="#fff2e3"
                v-model="repeatedPassword"
                variant="outlined"
                rounded="xl"  
                :type="showPassword ? 'text' : 'password'"

                
              >  
                <template v-slot:append-inner>
                  <v-icon @click="toggleVisibility" class="icon-size">{{showPassword ? 'mdi-eye' : 'mdi-eye-off'}}</v-icon>
                </template>


                <template v-slot:label>
                  <div style="font-size:20px; font-family:Verdana, Geneva, Tahoma, sans-serif; padding-left: 30px;">Repite la contraseña</div>
                 </template>
                
              </v-text-field>
            
              <v-text-field
               
                class="login-textfield"
                label="Telegram token"
                rounded="xl"
                bg-color="#fff2e3"
                v-model="token"
                variant="outlined"
              >

              <template v-slot:label>
                <div style="font-size:20px; font-family:Verdana, Geneva, Tahoma, sans-serif; padding-left: 30px;">Token de telegram</div>  
              </template>

              <template v-slot:append>
                <v-btn
                  color=#4e112e
                  href="https://t.me/PiWatch_AlertBot"
                  rel="noopener noreferrer"
                  target="_blank"
                  rounded="pill"
                  variant="flat"
                  >Consiga su token </v-btn>
                </template>

              <template v-slot:style>
                { "font-size: 20px" }
              </template>
            </v-text-field>
            
            <v-btn
              color="#4e112e"
              rounded="xl"
              @click="register"
              :loading="loading"
              width="99%"
              style="font-weight: bold; font-size: xx-large; margin-left: 1%; height: 20%;"
            >
              Registrarse
            </v-btn>
          
          
        
      </v-container>
    </v-row>
  </div>
</template>


<style scoped>


.icon-size{
  font-size: 44px;
  margin-right: 20px;
}
  .fade-background{
    background: linear-gradient(70deg, #f3f4ca, rgb(57, 2, 48));
  }
  .title {
  font-family: Georgia, 'Times New Roman', Times, serif;
  font-size: 60px; 
  color: #f3f4ca; 
  text-align: center; 
  margin-top: 20px; 
  text-transform: uppercase; 
  letter-spacing: 2px; 
  font-weight:bolder; 
}


.subtitle {
  font-family:'Lucida Sans', 'Lucida Sans Regular', 'Lucida Grande', 'Lucida Sans Unicode', Geneva, Verdana, sans-serif;
  font-size: 33px; 
  color: #f3f4ca; 
  text-align: center; 
  margin: 10px 0; 
  font-weight:bolder; 
}
  
  .v-container.component-card {
    background-color: #f3f4ca; 
    border-radius: 50px; 
    padding-top: 15%; 
    padding-left: 20px;
    padding-right: 20px;
    padding-bottom: 5%;
    margin: 40px;
    color: #300118;
    width: 40%;
    height: 80%;
    align-items: center;
    display: flex;
    justify-items: center;
    flex-direction: column;
    
  }

  .v-card.title-card {
    background-color: #4e112e;
    border-radius: 50px; 
    padding: 50px;
    margin: 10px;
    color: #2F2252;
    width: 40%;
    height: 80%;
    justify-items: center;
    align-items: center;
    display: flex;
    flex-direction: column;


  }

  .centered-container {
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100vh; 
    flex-direction: column;
  }

  .v-card.error{
    display: block; 
    position: fixed; 
    top: 2%; 
    background-color: #AB0012; 
    color: white; 
    padding: 30px;
    width: 40%;
    justify-items: center;
    align-items: center;
  }

  .v-text-field.login-textfield{
    height: 10%;
    width:95%;
    font-size: 20px !important;
    
  }
  
  


.fade-enter-active {
  transition: all 0.5s ease-out;
}

.fade-leave-active {
  transition: all 1s cubic-bezier(1, 0.5, 0.8, 1);
}

.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}
  

</style>

<script>
import axios from 'axios';
export default{
    data: () => ({
        loading: false,
        username: '',
        password: '',
        mostrarError: false,
        showPassword: false,
        repeatedPassword:'',
        token:''
    }),
    methods: {
        register() {
            this.loading = true;
            
            axios.post('http://localhost:8000/signin', { username: this.username, password: this.password, token:this.token })
                .then(response => {
                if (response.status === 200) {
                    this.loginSuccess = true;
                    this.loginError = null;
                    this.loading = false;
                    this.$router.push(`/schedules/${this.username}/`);
                }
                else {
                    this.loginSuccess = false;
                    this.loginError = 'Usuario o contraseña no coincidentes';
                    alert(this.loginError);
                }
            })
                .catch(error => {
                this.loginSuccess = false;
                this.loginError = 'Error en la solicitud';
                this.showError();
                this.loading = false;
            });
        },
        showError() {
            this.mostrarError = true;
            setTimeout(() => {
                this.mostrarError = false;
            }, 3000);
        },
        toggleVisibility(){
          this.showPassword=!this.showPassword;
        }
    },
  }


</script>