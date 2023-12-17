// Composables
import { createRouter, createWebHistory } from 'vue-router'
import login from '@/views/loginView';
import home from '@/views/homeView';
import main from '@/views/mainView';
import register from '@/views/registerView'
import incident from '@/views/incidentView';
import loginAttempt from '@/views/loginAttemptView'

const routes = [
  {
    path: '/schedules/:user',
    component: home
  },

  {
    path: '/',
    component: login
  },

  {
    path: '/home/:user',
    component: main
  },

  {
    path: '/loginAttempts/:user',
    component: loginAttempt
  },

  {
    path: '/register',
    component: register
  },

  {
    path: '/incidents/:user',
    component: incident
  },
]

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
})

export default router
