<template>
  <div class="dashboard-container">
    <r-o-s-topic />
    <!--    <component :is="currentRole" />-->
  </div>
</template>

<script>
import { mapGetters } from 'vuex'
import adminDashboard from './admin'
import editorDashboard from './editor'
import ROSTopic from '@/components/ROSComponents/ROStopic'

export default {
  name: 'Dashboard',
  components: { ROSTopic, adminDashboard, editorDashboard },
  data() {
    return {
      currentRole: 'adminDashboard'
    }
  },
  computed: {
    ...mapGetters([
      'roles'
    ])
  },
  created() {
    if (!this.roles.includes('admin')) {
      this.currentRole = 'editorDashboard'
    }
  }
}
</script>
