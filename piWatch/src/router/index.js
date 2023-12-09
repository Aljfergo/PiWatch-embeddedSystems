// Composables
import { createRouter, createWebHistory } from 'vue-router'
import login from '@/views/loginView';
import home from '@/views/homeView';
import main from '@/views/mainView';
import register from '@/views/registerView'

const routes = [
  {
    path: '/schedules/:user',
    component: home
  },

  {
    path: '/login',
    component: login
  },

  {
    path: '/main',
    component: main
  },

  {
    path: '/register',
    component: register
  },
]

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
})

export default router
