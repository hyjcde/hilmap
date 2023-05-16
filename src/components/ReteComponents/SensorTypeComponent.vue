<template>
  <div class="sensor-type-component">
    <h5>Sensor Type</h5>
    <el-select v-model="value" placeholder="Select Sensor Type">
      <el-option label="IMU" value="imu" />
      <el-option label="GPS" value="gps" />
      <el-option label="Lidar" value="lidar" :disabled="true" />
      <el-option label="Camera" value="camera" :disabled="true" />
      <el-option label="Radar" value="radar" :disabled="true" />
    </el-select>
  </div>
</template>

<script>
export default {
  name: 'SensorTypeComponent',
  data() {
    return {
      value: ''
    }
  },
  watch: {
    value() {
      this.$emit('value-change', this.value)
    }
  }
}
</script>

<style scoped>
.sensor-type-component {
  width: 100%;
}
</style>
