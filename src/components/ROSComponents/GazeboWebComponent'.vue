<template>
  <div>
    <div id="gazeboweb-container" />
  </div>
</template>

<script>
import * as ROSLIB from 'roslib'
import * as GAZEBO from 'gazebojs'

export default {
  data() {
    return {
      gzclient: null
    }
  },
  mounted() {
    this.initGazeboWeb()
  },
  methods: {
    initGazeboWeb() {
      // Connect to ROS
      const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
      })

      // Initialize Simulation client
      this.gzclient = new GAZEBO.GZ3D.Scene({
        container: 'gazeboweb-container',
        antialias: true
      })

      // Connect Simulation client to ROS
      this.gzclient.init(ros)
    }
  }
}
</script>

<style scoped>
#gazeboweb-container {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}
</style>
