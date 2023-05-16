<template>
  <div class="app-container">
    <aside>
      Welcome to the Fundamental System! <br>
      This guide will take you through the process of selecting modules, compiling code, and executing a task. <br><br>

      Step 1: Select Modules<br>
      Click on the "Modules" menu in the sidebar to view the available modules. Click on a module to select it.<br><br>

      Step 2: Compile Code<br>
      Once you have selected the desired modules, click on the "Compile" button to compile the code. This may take a few minutes, depending on the size of the project.<br><br>

      Step 3: Execute Task<br>
      After the code has been compiled, click on the "Execute Task" button to run the task on your system. You can monitor the progress of the task using the "Task Status" panel in the bottom right corner of the screen.<br><br>

      Click the button below to start the guided tour.
    </aside>
    <el-button
      icon="el-icon-question"
      type="primary"
      @click.prevent.stop="guide"
    >
      Show Guide
    </el-button>
  </div>
</template>

<script>
import Driver from 'driver.js' // import driver.js
import 'driver.js/dist/driver.min.css' // import driver.js css
import steps from './steps'

export default {
  name: 'Guide',
  data() {
    return {
      driver: null
    }
  },
  mounted() {
    this.driver = new Driver()
  },
  methods: {
    guide() {
      this.driver.defineSteps(steps)
      this.driver.start()
    }
  }
}
</script>
