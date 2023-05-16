<template>
  <div>
    <el-card class="overview-container">
      <h1 class="text-2xl font-semibold mb-6">
        <!--        巡检功能模块配置-->
        Module Selection
      </h1>
      <module-selection-diagram @submit-modules="updateFormWithSelectedModules" />
    </el-card>
  </div>
</template>

<script>
import roslib from 'roslib/src/core'
import ModuleSelectionDiagram from '@/components/ModuleSelection/ModuleSelectionDiagram'
import axios from 'axios'
export default {
  name: 'ModuleSelection',
  components: {
    ModuleSelectionDiagram
  },
  data() {
    return {
      form: {
        uavType: '',
        sensorType: '',
        venue: '',
        motionPlanner: '',
        taskModule: '',
        perceptionModule: '',
        ros: new roslib.Ros()
      }
    }
  },
  mounted() {
    this.fetchData()
    this.ros.connect('ws://172.245.79.240:9090') // Change the address as needed
  },
  methods: {
    fetchData() {
      console.log('getting data...')
      // Fetch data from Python backend
      axios.get('http://localhost:5001/api/hello')
        .then(response => {
          console.log('server:hello' + response.data.message)
        })
        .catch(error => {
          console.error('Error fetching data:', error)
        })
    },
    updateFormWithSelectedModules(selectedModules) {
      console.log('Used Modules:', selectedModules)

      if (selectedModules.uav) {
        this.form.uavType = selectedModules.uav.name
      }
      if (selectedModules.sensor) {
        this.form.sensorType = selectedModules.sensor.name
      }
      if (selectedModules.venue) {
        this.form.venue = selectedModules.venue.name
      }
      if (selectedModules.motionPlanner) {
        this.form.motionPlanner = selectedModules.motionPlanner.name
      }
      if (selectedModules.task) {
        this.form.taskModule = selectedModules.task.name
      }
      if (selectedModules.perception) {
        this.form.perceptionModule = selectedModules.perception.name
      }

      // Call the submit function after updating the form with the selected modules
      this.submit()
    },

    submit() {
      console.log('Submitt Modules:', this.form)
      if (this.validateForm()) {
        this.setROSParams()
        this.$emit('submit', this.form)
      }
    },
    async setROSParams() {
      const params = {
        robot_choice: this.form.uavType,
        task_planner_choice: this.form.taskModule,
        workspace_choice: this.form.venue
      }

      try {
        const response = await axios.post('http://localhost:5001/api/launch_remote_planner', params)
        console.log('Response from Flask API:', response.data)
      } catch (error) {
        console.error('Error calling Flask API:', error)
      }
    },
    validateForm() {
      const requiredFields = [
        { field: 'uavType', label: 'UAV Type' },
        { field: 'sensorType', label: 'Sensor Type' },
        { field: 'venue', label: 'Venue' },
        { field: 'motionPlanner', label: 'Motion Planner' },
        { field: 'taskModule', label: 'Task Module' },
        { field: 'perceptionModule', label: 'Perception Module' }
      ]

      for (const { field, label } of requiredFields) {
        if (!this.form[field] || (Array.isArray(this.form[field]) && this.form[field].length === 0)) {
          this.$message.warning(`Please select a ${label}`)
          return false
        }
      }

      return true
    }

  }

}

</script>

<style scoped>

.overview-container {
  margin: 25px;
  width: 96vw;
  height: 90vh;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1), 0 1px 3px rgba(0, 0, 0, 0.08);
  border-radius: 4px;
  padding: .5rem;
  align-content: center;
  text-align: center;
}

.h1 {
  text-align: center;
  font-size: 28px;
  font-weight: bold;
  margin-bottom: 20px;
}

.el-form-item__label {
  font-size: 16px;
  font-weight: bold;
}

.el-input {
  width: 100%;
}

.el-button {
  margin-top: 10px;
}

.el-radio-button {
  padding: 5px 20px;
  border-radius: 20px;
  font-size: 14px;
}

.el-checkbox {
  font-size: 14px;
}

.el-checkbox__label {
  padding-left: 10px;
}

@media only screen and (max-width: 576px) {

  .el-form-item__label {
    font-size: 14px;
  }

  .el-radio-button {
    font-size: 12px;
    padding: 5px 10px;
  }

  .el-checkbox {
    font-size: 12px;
  }
}
</style>

